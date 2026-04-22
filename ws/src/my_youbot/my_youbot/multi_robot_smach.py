#!/usr/bin/env python3
"""
multi_robot_smach.py
Реализация сети Петри (3 робота, 2 семафора) через иерархические
SMACH-машины состояний и point-to-point контроллер (Pose2D goal).
"""

import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose2D
import smach

# ─────────────────────────────────────────────
#  Глобальный ROS2-узел (один на весь процесс)
# ─────────────────────────────────────────────
class MultiRobotSmach(Node):
    def __init__(self):
        super().__init__('multi_robot_smach')

        # Семафоры (фишки) — по одной фишке на флаг
        self.sem1 = threading.Semaphore(1)   # флаг готовности 1 (robot_0 + robot_1)
        self.sem2 = threading.Semaphore(1)   # флаг готовности 2 (robot_1 + robot_2)

        # Словарь: имя робота → имя флага
        self.robot_semaphores = {
            'my_robot_0': self.sem1,
            'my_robot_1': self.sem1,   # youbot_2 делит оба флага —
            'my_robot_2': self.sem2,   #   здесь упрощение: каждый робот
        }                              #   использует свой «правый» флаг

        # Публикаторы цели для каждого робота
        # point_to_point_controller слушает /<robot_name>/goal_pose (Pose2D)
        self.goal_pubs = {
            name: self.create_publisher(Pose2D, f'/{name}/goal_pose', 1)
            for name in ('my_robot_0', 'my_robot_1', 'my_robot_2')
        }

        # Словарь: флаги «цель достигнута» (обновляются из коллбэков)
        self.goal_reached = {name: False
                             for name in ('my_robot_0', 'my_robot_1', 'my_robot_2')}

        # Подписки на /my_robot_N/goal_reached (Bool, публикует p2p-контроллер)
        for name in ('my_robot_0', 'my_robot_1', 'my_robot_2'):
            self.create_subscription(
                Bool,
                f'/{name}/goal_reached',
                lambda msg, n=name: self._goal_cb(msg, n),
                1
            )

    def _goal_cb(self, msg: Bool, robot_name: str):
        if msg.data:
            self.goal_reached[robot_name] = True

    def send_goal(self, robot_name: str, x: float, y: float, yaw: float):
        """Публикует цель и сбрасывает флаг достижения."""
        self.goal_reached[robot_name] = False
        goal = Pose2D(x=x, y=y, theta=yaw)
        self.goal_pubs[robot_name].publish(goal)

    def wait_for_goal(self, robot_name: str, timeout_sec: float = 30.0):
        """Блокирует поток пока p2p не сообщит о достижении цели."""
        import time
        t0 = time.time()
        while not self.goal_reached[robot_name]:
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.time() - t0 > timeout_sec:
                self.get_logger().warn(f'{robot_name}: goal timeout!')
                break


# ─────────────────────────────────────────────
#  Точки маршрута для каждого робота
#  (x, y, yaw) — координаты Webots-мира
#  Скорректируйте под реальное расположение стен
# ─────────────────────────────────────────────
WAYPOINTS = {
    'my_robot_0': {
        'entry':  ( 1.0,  0.5, -1.5708),   # Въезд в коридор (стена wall 0)
        'mid':    ( 1.0, -1.5, -1.5708),   # Преодоление коридора
        'exit':   ( 1.0, -3.0, -1.5708),   # Выезд из коридора
        'wait':   ( 2.0,  2.5,  1.5708),   # Точка ожидания (старт)
    },
    'my_robot_1': {
        'entry':  ( 0.0,  0.5, -1.5708),
        'mid':    ( 0.0, -1.5, -1.5708),
        'exit':   ( 0.0, -3.0, -1.5708),
        'wait':   ( 0.0,  2.5,  1.5708),
    },
    'my_robot_2': {
        'entry':  (-2.0,  0.5, -1.5708),
        'mid':    (-2.0, -1.5, -1.5708),
        'exit':   (-2.0, -3.0, -1.5708),
        'wait':   (-2.0,  2.5,  1.5708),
    },
}


# ─────────────────────────────────────────────
#  Состояния SMACH
# ─────────────────────────────────────────────

class WaitStart(smach.State):
    """Начальное ожидание — аналог верхнего места сети Петри."""
    def __init__(self, node, robot_name):
        super().__init__(outcomes=['request_token'])
        self.node = node
        self.robot_name = robot_name

    def execute(self, userdata):
        self.node.get_logger().info(f'[{self.robot_name}] WaitStart')
        rclpy.spin_once(self.node, timeout_sec=0.5)
        return 'request_token'


class AcquireToken(smach.State):
    """Запрос фишки у семафора (блокирующий — реализует дугу «запрос фишки»)."""
    def __init__(self, node, robot_name, semaphore):
        super().__init__(outcomes=['acquired'])
        self.node = node
        self.robot_name = robot_name
        self.sem = semaphore

    def execute(self, userdata):
        self.node.get_logger().info(
            f'[{self.robot_name}] Waiting for token...')
        self.sem.acquire()   # блокирует, пока фишка не свободна
        self.node.get_logger().info(
            f'[{self.robot_name}] Token acquired!')
        return 'acquired'


class EnterCorridor(smach.State):
    """Въезд в коридор — p2p к точке entry."""
    def __init__(self, node, robot_name):
        super().__init__(outcomes=['done'])
        self.node = node
        self.robot_name = robot_name

    def execute(self, userdata):
        wp = WAYPOINTS[self.robot_name]['entry']
        self.node.get_logger().info(
            f'[{self.robot_name}] EnterCorridor → {wp}')
        self.node.send_goal(self.robot_name, *wp)
        self.node.wait_for_goal(self.robot_name)
        return 'done'


class TraverseCorridor(smach.State):
    """Преодоление коридора — p2p к точке mid."""
    def __init__(self, node, robot_name):
        super().__init__(outcomes=['done'])
        self.node = node
        self.robot_name = robot_name

    def execute(self, userdata):
        wp = WAYPOINTS[self.robot_name]['mid']
        self.node.get_logger().info(
            f'[{self.robot_name}] TraverseCorridor → {wp}')
        self.node.send_goal(self.robot_name, *wp)
        self.node.wait_for_goal(self.robot_name)
        return 'done'


class ExitCorridor(smach.State):
    """Выезд из коридора — p2p к точке exit."""
    def __init__(self, node, robot_name):
        super().__init__(outcomes=['done'])
        self.node = node
        self.robot_name = robot_name

    def execute(self, userdata):
        wp = WAYPOINTS[self.robot_name]['exit']
        self.node.get_logger().info(
            f'[{self.robot_name}] ExitCorridor → {wp}')
        self.node.send_goal(self.robot_name, *wp)
        self.node.wait_for_goal(self.robot_name)
        return 'done'


class ReleaseToken(smach.State):
    """Возврат фишки семафору (дуга «возврат фишки» на схеме)."""
    def __init__(self, node, robot_name, semaphore):
        super().__init__(outcomes=['released'])
        self.node = node
        self.robot_name = robot_name
        self.sem = semaphore

    def execute(self, userdata):
        # Возвращаем робота в точку ожидания
        wp = WAYPOINTS[self.robot_name]['wait']
        self.node.send_goal(self.robot_name, *wp)
        self.node.wait_for_goal(self.robot_name)
        self.sem.release()
        self.node.get_logger().info(
            f'[{self.robot_name}] Token released')
        return 'released'


class WaitIdle(smach.State):
    """Ожидание перед следующим циклом (нижнее место сети Петри)."""
    def __init__(self, node, robot_name):
        super().__init__(outcomes=['restart'])
        self.node = node
        self.robot_name = robot_name

    def execute(self, userdata):
        self.node.get_logger().info(f'[{self.robot_name}] WaitIdle')
        rclpy.spin_once(self.node, timeout_sec=1.0)
        return 'restart'


# ─────────────────────────────────────────────
#  Сборка FSM для одного робота
# ─────────────────────────────────────────────
def build_robot_sm(node: MultiRobotSmach, robot_name: str) -> smach.StateMachine:
    sem = node.robot_semaphores[robot_name]

    # Внутренняя подмашина «коридор» (Въезд→Преодоление→Выезд)
    corridor_sm = smach.StateMachine(outcomes=['corridor_done'])
    with corridor_sm:
        smach.StateMachine.add(
            'ENTER', EnterCorridor(node, robot_name),
            transitions={'done': 'TRAVERSE'})
        smach.StateMachine.add(
            'TRAVERSE', TraverseCorridor(node, robot_name),
            transitions={'done': 'EXIT'})
        smach.StateMachine.add(
            'EXIT', ExitCorridor(node, robot_name),
            transitions={'done': 'corridor_done'})

    # Верхняя машина (цикличная)
    sm = smach.StateMachine(outcomes=['loop_end'])
    with sm:
        smach.StateMachine.add(
            'WAIT_START', WaitStart(node, robot_name),
            transitions={'request_token': 'ACQUIRE_TOKEN'})
        smach.StateMachine.add(
            'ACQUIRE_TOKEN', AcquireToken(node, robot_name, sem),
            transitions={'acquired': 'CORRIDOR'})
        smach.StateMachine.add(
            'CORRIDOR', corridor_sm,
            transitions={'corridor_done': 'RELEASE_TOKEN'})
        smach.StateMachine.add(
            'RELEASE_TOKEN', ReleaseToken(node, robot_name, sem),
            transitions={'released': 'WAIT_IDLE'})
        smach.StateMachine.add(
            'WAIT_IDLE', WaitIdle(node, robot_name),
            transitions={'restart': 'WAIT_START'})   # ← цикл
    return sm


# ─────────────────────────────────────────────
#  Запуск трёх FSM в параллельных потоках
# ─────────────────────────────────────────────
def run_robot(node: MultiRobotSmach, robot_name: str):
    sm = build_robot_sm(node, robot_name)
    sm.execute()


def main(args=None):
    rclpy.init(args=args)
    node = MultiRobotSmach()

    threads = []
    for name in ('my_robot_0', 'my_robot_1', 'my_robot_2'):
        t = threading.Thread(target=run_robot, args=(node, name), daemon=True)
        threads.append(t)
        t.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()