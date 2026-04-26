#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import smach
import threading
import time

from my_youbot_interfaces.action import MoveToPose

# ------------------------------------------------------------
# Глобальные настройки
# ------------------------------------------------------------
# доступные коридоры: [левый, правый]
available_corridor = [1, 1]
corridor_lock = threading.Lock()

# тут хранятся ИНДЕКСЫ флагов для проверки соответствующих коридоров в массиве available_corridor. 
# Индекс "-1" обозначает, что для проезда по коридору не требуется проверка индекса его свободы (этот коридор занимает только один робот)
ROBOT_CORRIDOR_INDEX = {
    'my_robot_0': {'left': -1, 'right':  0},
    'my_robot_1': {'left':  0, 'right':  1},
    'my_robot_2': {'left':  1, 'right': -1},
}

BASE_ROUTE = [
    ( 0.0,  2.5,  0.0   ),   # 0 - старт/финиш снизу
    ( 1.0,  2.5, -1.5708),   # 1 - вход в левый коридор
    ( 1.0, -2.5, -1.5708),   # 2 - выход из левого коридора
    ( 0.0, -2.5,  3.1415),   # 3 - ожидание сверху
    (-1.0, -2.5,  1.5708),   # 4 - вход в правый коридор
    (-1.0,  2.5,  1.5708),   # 5 - выход из правого коридора
]

OFFSETS = {
    'my_robot_0':  1.0,
    'my_robot_1': -1.0,
    'my_robot_2': -3.0,
}

# ------------------------------------------------------------
# Action клиент с возможностью ожидания завершения
# ------------------------------------------------------------
class RobotActionClient:
    def __init__(self, node: Node, robot_name: str):
        self.node = node
        self.robot_name = robot_name
        self._action_client = ActionClient(node, MoveToPose, f'/{robot_name}/navigate_to_pose')
        self._goal_handle = None
        self._result_future = None
        self._is_busy = False
        self._success = False

    def send_goal_and_wait(self, target_point, timeout=30.0):
        """Отправить goal и дождаться результата (блокирующий вызов, но с spin_once)."""
        if self._is_busy:
            self.node.get_logger().warn(f"[{self.robot_name}] уже выполняет движение")
            return False

        goal_msg = MoveToPose.Goal()
        goal_msg.target.x = target_point[0]
        goal_msg.target.y = target_point[1]
        goal_msg.target.theta = target_point[2]
        self.node.get_logger().info(f"[{self.robot_name}] отправляю точку: ({goal_msg.target.x}, {goal_msg.target.y}, {goal_msg.target.theta:.2f})")

        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error(f"[{self.robot_name}] action сервер не доступен")
            return False

        # Асинхронная отправка
        send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self._feedback_callback)
        # Блокируемся до получения ответа от сервера
        rclpy.spin_until_future_complete(self.node, send_goal_future, timeout_sec=timeout)
        if not send_goal_future.done():
            self.node.get_logger().error(f"[{self.robot_name}] таймаут при отправке goal")
            return False

        self._goal_handle = send_goal_future.result()
        if not self._goal_handle.accepted:
            self.node.get_logger().warn(f"[{self.robot_name}] goal отклонён")
            self._goal_handle = None
            return False

        self.node.get_logger().info(f"[{self.robot_name}] goal принят, жду выполнения...")
        self._is_busy = True
        self._success = False

        # Ждём результат
        self._result_future = self._goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, self._result_future, timeout_sec=timeout)
        if not self._result_future.done():
            self.node.get_logger().error(f"[{self.robot_name}] таймаут при ожидании результата")
            self._is_busy = False
            return False

        result = self._result_future.result().result
        self._success = result.success
        if self._success:
            self.node.get_logger().info(f"[{self.robot_name}] движение успешно завершено")
        else:
            self.node.get_logger().warn(f"[{self.robot_name}] движение не удалось")
        self._is_busy = False
        return self._success

    def _feedback_callback(self, feedback_msg):
        # Необязательная обратная связь
        current_pose = feedback_msg.feedback.current_pose
        self.node.get_logger().debug(f"[{self.robot_name}] обратная связь: {current_pose}")

# ------------------------------------------------------------
# Состояния SMACH
# ------------------------------------------------------------
class GoToPoint(smach.State):
    """Состояние движения к точке – дожидается завершения действия."""
    def __init__(self, robot_client: RobotActionClient, target_point):
        super().__init__(outcomes=['NEXT'])
        self.client = robot_client
        self.target = target_point

    def execute(self, ud):
        success = self.client.send_goal_and_wait(self.target)
        if success:
            return 'NEXT'
        else:
            # Можно добавить повтор или переход в ошибку, пока просто NEXT
            self.client.node.get_logger().error(f"[{self.client.robot_name}] не удалось достичь точки, идём дальше?")
            return 'NEXT'

class Waiting(smach.State):
    """Ожидание освобождения коридора. При успешном захвате переходит NEXT."""
    def __init__(self, robot_client: RobotActionClient, route_side: str):
        """
        :param route_side: 'left' или 'right' – какой коридор проверяем
        """
        super().__init__(outcomes=['NEXT', 'WAIT'])
        self.client = robot_client
        self.route_side = route_side   # 'left' или 'right'
        self.previous_corridor_state = 1

    def execute(self, ud):
        robot = self.client.robot_name
        corridor_idx = ROBOT_CORRIDOR_INDEX[robot][self.route_side]  # -1, 0 или 1
        with corridor_lock:
            if corridor_idx != -1:
                if available_corridor[corridor_idx] == 1:
                    # Коридор свободен – захватываем
                    available_corridor[corridor_idx] = 0
                    self.previous_corridor_state = 1

                    self.client.node.get_logger().info(f"[{robot}] захватил коридор {self.route_side} (индекс {corridor_idx})")
                    return 'NEXT'
                else:
                    if self.previous_corridor_state != available_corridor[corridor_idx]:
                        self.previous_corridor_state = available_corridor[corridor_idx]
                        self.client.node.get_logger().info(f"[{robot}] ждёт освобождения коридора {self.route_side}")
                    return 'WAIT'
            else:
                return 'NEXT'

class ReleaseCorridor(smach.State):
    """Освобождение ранее занятого коридора."""
    def __init__(self, robot_client: RobotActionClient, route_side: str):
        super().__init__(outcomes=['NEXT'])
        self.client = robot_client
        self.route_side = route_side

    def execute(self, ud):
        robot = self.client.robot_name
        corridor_idx = ROBOT_CORRIDOR_INDEX[robot][self.route_side]
        with corridor_lock:
            if corridor_idx != -1:
                available_corridor[corridor_idx] = 1
        self.client.node.get_logger().info(f"[{robot}] освободил коридор {self.route_side} (индекс {corridor_idx})")
        return 'NEXT'

# ------------------------------------------------------------
# Менеджер нескольких роботов
# ------------------------------------------------------------
class MultiRobotManager(Node):
    def __init__(self):
        super().__init__('multi_robot_manager')
        self.declare_parameter('robot_names', ['my_robot_0', 'my_robot_1', 'my_robot_2'])
        self.robot_names = self.get_parameter('robot_names').value

        # Строим concurrence
        self.concurrence = smach.Concurrence(
            outcomes=['END'],
            default_outcome='END',
            outcome_map={'END': {f'{name}_sm': 'END' for name in self.robot_names}}
        )

        with self.concurrence:
            for r_name in self.robot_names:
                robot_sm = self.create_robot_sm(r_name)
                self.concurrence.add(f'{r_name}_sm', robot_sm) 

        self.get_logger().info("Concurrence построена, запускаем в отдельном потоке")
        # Запускаем Concurrence в фоновом потоке, чтобы не блокировать spin()
        self.smach_thread = threading.Thread(target=self._run_concurrence)
        self.smach_thread.daemon = True
        self.smach_thread.start()

    def _run_concurrence(self):
        """Запуск SMACH автомата (блокирующий вызов)."""
        outcome = self.concurrence.execute()
        self.get_logger().info(f"Concurrence завершилась с исходом: {outcome}")
        # После завершения всех роботов можно выключить ноду
        rclpy.shutdown()

    def create_robot_sm(self, robot_name):
        """Создаёт StateMachine для одного робота."""
        # Вычисляем маршрут со смещением
        route = [[x + OFFSETS[robot_name], y, angle] for (x, y, angle) in BASE_ROUTE]
        client = RobotActionClient(self, robot_name)

        sm = smach.StateMachine(outcomes=['END', 'loop_done'])  # loop_done для внутреннего цикла
        with sm:
            # ----- нижняя часть (левый коридор) -----
            smach.StateMachine.add(
                'WAIT_BOTTOM',
                Waiting(client, route_side='left'),
                transitions={'NEXT': 'GO_INTO_LEFT', 'WAIT': 'WAIT_BOTTOM'}
            )
            smach.StateMachine.add(
                'GO_INTO_LEFT',
                GoToPoint(client, route[1]),
                transitions={'NEXT': 'GO_THROUGH_LEFT'}
            )
            smach.StateMachine.add(
                'GO_THROUGH_LEFT',
                GoToPoint(client, route[2]),
                transitions={'NEXT': 'GO_OUT_LEFT'}
            )
            smach.StateMachine.add(
                'GO_OUT_LEFT',
                GoToPoint(client, route[3]),
                transitions={'NEXT': 'RELEASE_LEFT'}
            )
            smach.StateMachine.add(
                'RELEASE_LEFT',
                ReleaseCorridor(client, route_side='left'),
                transitions={'NEXT': 'WAIT_TOP'}
            )

            # ----- верхняя часть (правый коридор) -----
            smach.StateMachine.add(
                'WAIT_TOP',
                Waiting(client, route_side='right'),
                transitions={'NEXT': 'GO_INTO_RIGHT', 'WAIT': 'WAIT_TOP'}
            )
            smach.StateMachine.add(
                'GO_INTO_RIGHT',
                GoToPoint(client, route[4]),
                transitions={'NEXT': 'GO_THROUGH_RIGHT'}
            )
            smach.StateMachine.add(
                'GO_THROUGH_RIGHT',
                GoToPoint(client, route[5]),
                transitions={'NEXT': 'GO_OUT_RIGHT'}
            )
            smach.StateMachine.add(
                'GO_OUT_RIGHT',
                GoToPoint(client, route[0]),  # возврат в начало
                transitions={'NEXT': 'RELEASE_RIGHT'}
            )
            smach.StateMachine.add(
                'RELEASE_RIGHT',
                ReleaseCorridor(client, route_side='right'),
                transitions={'NEXT': 'WAIT_BOTTOM'}   # зацикливание
            )

        return sm

# ------------------------------------------------------------
# Основная функция
# ------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = MultiRobotManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()