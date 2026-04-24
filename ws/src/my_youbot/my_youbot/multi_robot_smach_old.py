#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import smach
import threading
import time

from my_youbot_interfaces.action import MoveToPose

class SemaphoreManager:
    def __init__(self):
        self.locks = {
            'corridor_0': threading.Lock(),
            'corridor_2': threading.Lock()
        }
    def acquire(self, corridor_name, timeout=None):
        return self.locks[corridor_name].acquire(timeout=timeout)
    def release(self, corridor_name):
        if self.locks[corridor_name].locked():
            self.locks[corridor_name].release()
    def is_locked(self, corridor_name):
        return self.locks[corridor_name].locked()

BASE_ROUTE = [
    ( 0.0,  2.5,  0.0   ),
    ( 1.0,  2.5, -1.5708),
    ( 1.0, -2.5, -1.5708),
    ( 0.0, -2.5,  3.1415),
    (-1.0, -2.5,  1.5708),
    (-1.0,  2.5,  1.5708),
]

OFFSETS = {
    'my_robot_0':  1.0,
    'my_robot_1': -1.0,
    'my_robot_2': -3.0,
}

class RobotActionClient:
    def __init__(self, node, robot_namespace):
        self.node = node
        self.client = ActionClient(node, MoveToPose, f'/{robot_namespace}/navigate_to_pose')
        self.namespace = robot_namespace

    def send_goal(self, x, y, yaw, timeout=30.0):
        goal_msg = MoveToPose.Goal()
        goal_msg.target.x = x
        goal_msg.target.y = y
        goal_msg.target.theta = yaw
        self.node.get_logger().info(f"[{self.namespace}] Sending goal: ({x}, {y}, {yaw:.2f})")

        if not self.client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error(f"[{self.namespace}] Action server not available")
            return False

        future = self.client.send_goal_async(goal_msg)
        done_event = threading.Event()
        future.add_done_callback(lambda fut: done_event.set())
        if not done_event.wait(timeout=timeout):
            self.node.get_logger().error(f"[{self.namespace}] Goal timeout")
            return False

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().error(f"[{self.namespace}] Goal rejected")
            return False

        result_future = goal_handle.get_result_async()
        done_event = threading.Event()
        result_future.add_done_callback(lambda fut: done_event.set())
        if not done_event.wait(timeout=timeout):
            self.node.get_logger().error(f"[{self.namespace}] Result timeout")
            return False

        result = result_future.result().result
        return result.success

class MoveAlongRoute(smach.State):
    def __init__(self, node, robot_name, route, semaphore_manager, action_client):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'],
                             input_keys=['route_index'],
                             output_keys=['route_index'])
        self.node = node
        self.robot_name = robot_name
        self.route = route
        self.sem = semaphore_manager
        self.client = action_client
        self.acquired_semaphores = []

    def _acquire_with_wait(self, sem_name):
        self.node.get_logger().info(f"{self.robot_name} waiting for {sem_name}")
        while not self.sem.acquire(sem_name, timeout=1.0):
            time.sleep(0.1)   # не используем spin_once, просто спим
        self.acquired_semaphores.append(sem_name)
        self.node.get_logger().info(f"{self.robot_name} acquired {sem_name}")

    def _release_if_held(self, sem_name):
        if sem_name in self.acquired_semaphores:
            self.sem.release(sem_name)
            self.acquired_semaphores.remove(sem_name)
            self.node.get_logger().info(f"{self.robot_name} released {sem_name}")

    def _release_all(self):
        for sem in list(self.acquired_semaphores):
            self._release_if_held(sem)

    def execute(self, userdata):
        idx = userdata.route_index
        try:
            while idx < len(self.route):
                x, y, yaw = self.route[idx]
                # Логика семафоров
                if self.robot_name == 'my_robot_0':
                    if idx == 2 and x == 1.0 and y == -2.5:
                        self._acquire_with_wait('corridor_0')
                    elif idx == 1 and x == 1.0 and y == 2.5:
                        self._release_if_held('corridor_0')
                elif self.robot_name == 'my_robot_1':
                    if idx == 0 and x == -1.0 and y == 2.5:
                        self._acquire_with_wait('corridor_0')
                        self._release_if_held('corridor_2')
                    elif idx == 3 and x == -1.0 and y == -2.5:
                        self._acquire_with_wait('corridor_2')
                        self._release_if_held('corridor_0')
                elif self.robot_name == 'my_robot_2':
                    if idx == 0 and x == -3.0 and y == 2.5:
                        self._acquire_with_wait('corridor_2')
                    elif idx == 3 and x == -3.0 and y == -2.5:
                        self._release_if_held('corridor_2')

                success = self.client.send_goal(x, y, yaw)
                if not success:
                    self._release_all()
                    return 'aborted'
                idx += 1
                userdata.route_index = idx
        except Exception as e:
            self.node.get_logger().error(f"{self.robot_name} exception: {e}")
            self._release_all()
            return 'aborted'

        self._release_all()
        return 'succeeded'

def create_robot_sm(node, robot_name, sem_manager):
    sm = smach.StateMachine(outcomes=['final'])
    offset_x = OFFSETS[robot_name]
    route = [(x + offset_x, y, yaw) for (x, y, yaw) in BASE_ROUTE]
    client = RobotActionClient(node, robot_name)
    with sm:
        sm.add('MOVE', MoveAlongRoute(node, robot_name, route, sem_manager, client),
               transitions={'succeeded':'final', 'aborted':'final', 'preempted':'final'})
    sm.userdata.route_index = 0
    return sm

class MultiRobotManager(Node):
    def __init__(self):
        super().__init__('multi_robot_manager')
        self.sem_manager = SemaphoreManager()
        self.robots = ['my_robot_0', 'my_robot_1', 'my_robot_2']
        self.smach_threads = []
        for robot in self.robots:
            sm = create_robot_sm(self, robot, self.sem_manager)
            t = threading.Thread(target=self.run_sm, args=(sm, robot))
            t.start()
            self.smach_threads.append(t)
        self.get_logger().info("MultiRobotManager started")

    def run_sm(self, sm, robot_name):
        self.get_logger().info(f"Starting state machine for {robot_name}")
        outcome = sm.execute()
        self.get_logger().info(f"{robot_name} finished with outcome: {outcome}")

def main(args=None):
    rclpy.init(args=args)
    node = MultiRobotManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()