#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import smach
# import threading
import time

from my_youbot_interfaces.action import MoveToPose



avaliable_corridor = [1, 1]     # левый, правый

# список флагов, которые необходимо проверять роботу
# возвращает массив, содержащий ИНДЕКСЫ массива avaliable_corridor для левого и правого флага относительно пути робота
ROBOT_FLAGS = {
    'my_robot_0': [-1,  0],
    'my_robot_1': [ 0,  1],
    'my_robot_2': [ 1, -1],
}

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


class Robot_ActionClient:
    def __init__(self, node, robot_name):
        self.node = node
        self._action_client = ActionClient(node, MoveToPose, f'/{robot_name}/navigate_to_pose')
        self.robot_name = robot_name
        self._goal_handle = None


    def send_goal(self, target_point, timeout=5.0):
        goal_msg = MoveToPose.Goal()
        goal_msg.target.x = target_point[0]     # x 
        goal_msg.target.y = target_point[1]     # y
        goal_msg.target.theta = target_point[2] # yaw
        self.node.get_logger().info(f"[{self.robot_name}] Sending goal: ({goal_msg.target.x}, {goal_msg.target.y}, {goal_msg.target.theta:.2f})")

        if not self._action_client.wait_for_server(timeout_sec=timeout):
            self.node.get_logger().error(f"[{self.namespace}] Action server not available")
            return #False

        # Асинхронная отправка
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)


    def goal_response_callback(self, future):   # Вызывается, когда сервер принял (или отклонил) goal.
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.get_logger().warn(f"[{self.namespace}] goal_response_callback: Задача отклонена сервером")
            self._goal_handle = None
            return

        self.get_logger().info("Задача ПРИНЯТА сервером!")

        # Ждём итог (Result)
        self._get_result_future = self._goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)


    def feedback_callback(self, feedback_msg):  # Вызывается при получении feedback от сервера.
        current_pose = feedback_msg.feedback.current_pose
        self.get_logger().info(f"[{self.namespace}] feedback: робот  находится в {current_pose}")


    def get_result_callback(self, future):  # Вызывается, когда сервер завершил задачу (succeed, abort, cancel).

        result = future.result().result
        if result.success:
            self.get_logger().info(f"Задача завершена УСПЕШНО!   Финальная точка робота: {result.final_pose}")
        else:
            self.get_logger().warn(f"Задача  завершена НЕУДАЧНО! Положение робота: {result.final_pose}")
        # Освободим goal_handle, чтобы можно было отправить новый
        self._goal_handle = None



class MultiRobotManager(Node):
    def __init__(self):
        super().__init__('multi_robot_manager')
        
        self.declare_parameter('robot_names', ['my_robot_0', 'my_robot_1', 'my_robot_2'])
        self.robot_names = self.get_parameter('robot_names').value

        cc = smach.Concurrence(outcomes=['END'],
                 default_outcome='END',
                 outcome_map={'END': {f'{name}_sm' : 'END' for name in self.robot_names}})
        
        with cc:
            for r_name in self.robot_names:
                smach.Concurrence.add(f'{r_name}_sm', self.create_robot_sm(r_name))

        self.get_logger().info("MultiRobotManager Concurrence state machine built")
        cc.execute()


    def create_robot_sm(self, robot_name):
        route = [[x + OFFSETS[robot_name], y, angle] for (x, y, angle) in BASE_ROUTE]
        client = Robot_ActionClient(self, robot_name)
        
        sm = smach.StateMachine(outcomes=['loop_done', 'END'])
        with sm:
            # smach.StateMachine.add(f'{robot_name}_waiting_bottom',          waiting(client, route_index = 0),   # 0 - левый коридор
            #                        transitions={'GO_TO'             :       f'{robot_name}_go_into_left_corridor',
            #                                     'WAIT'              :       f'{robot_name}_waiting_bottom'
            #                         })

            # smach.StateMachine.add(f'{robot_name}_go_into_left_corridor',   go_into_corridor(client, route[1]),
            #                        transitions={'GO_THROUGH'        :       f'{robot_name}_go_through_left_corridor',
            #                         })
            
            # smach.StateMachine.add(f'{robot_name}_go_through_left_corridor',go_into_corridor(client, route[2]),
            #                        transitions={'GO_OUT'            :       f'{robot_name}_go_out_left_corridor',
            #                         })
            
            # smach.StateMachine.add(f'{robot_name}_go_out_left_corridor',    go_into_corridor(client, route[3]),
            #                        transitions={'WAIT'              :       f'{robot_name}_waiting_top',
            #                         })

            # # ---------------------------------

            # smach.StateMachine.add(f'{robot_name}_waiting_top',             waiting(client, route_index = 1),   # 1 - правый коридор
            #                        transitions={'GO_TO'             :       f'{robot_name}_go_into_right_corridor',
            #                                     'WAIT'              :       f'{robot_name}_waiting_top'
            #                         })

            # smach.StateMachine.add(f'{robot_name}_go_into_right_corridor',   go_into_corridor(client, route[4]),
            #                        transitions={'GO_THROUGH'        :       f'{robot_name}_go_through_right_corridor',
            #                         })
            
            # smach.StateMachine.add(f'{robot_name}_go_through_right_corridor',go_into_corridor(client, route[5]),
            #                        transitions={'GO_OUT'            :       f'{robot_name}_go_out_right_corridor',
            #                         })
            
            # smach.StateMachine.add(f'{robot_name}_go_out_right_corridor',   go_into_corridor(client, route[0]),
            #                        transitions={'WAIT'              :       f'{robot_name}_waiting_bottom',
            #                         })

            smach.StateMachine.add(f'{robot_name}_waiting_bottom',          waiting(client, route_index = 0),   # 0 - левый коридор
                                   transitions={'NEXT'              :       f'{robot_name}_go_into_left_corridor',
                                                'WAIT'              :       f'{robot_name}_waiting_bottom'
                                    })

            smach.StateMachine.add(f'{robot_name}_go_into_left_corridor',   go_to_point(client, route[1]),
                                   transitions={'NEXT'              :             f'{robot_name}_go_through_left_corridor',
                                    })
            
            smach.StateMachine.add(f'{robot_name}_go_through_left_corridor',go_to_point(client, route[2]),
                                   transitions={'NEXT'              :       f'{robot_name}_go_out_left_corridor',
                                    })
            
            smach.StateMachine.add(f'{robot_name}_go_out_left_corridor',    go_to_point(client, route[3]),
                                   transitions={'NEXT'              :       f'{robot_name}_waiting_top',
                                    })

            # ---------------------------------

            smach.StateMachine.add(f'{robot_name}_waiting_top',             waiting(client, route_index = 1),   # 1 - правый коридор
                                   transitions={'NEXT'              :       f'{robot_name}_go_into_right_corridor',
                                                'WAIT'              :       f'{robot_name}_waiting_top'
                                    })

            smach.StateMachine.add(f'{robot_name}_go_into_right_corridor',   go_to_point(client, route[4]),
                                   transitions={'NEXT'              :       f'{robot_name}_go_through_right_corridor',
                                    })
            
            smach.StateMachine.add(f'{robot_name}_go_through_right_corridor',go_to_point(client, route[5]),
                                   transitions={'NEXT'              :       f'{robot_name}_go_out_right_corridor',
                                    })
            
            smach.StateMachine.add(f'{robot_name}_go_out_right_corridor',   go_to_point(client, route[0]),
                                   transitions={'NEXT'              :       f'{robot_name}_waiting_bottom',
                                   })

        return sm



class go_to_point(smach.State):
    def __init__(self, robot_client, target_point):
        super().__init__(outcomes=['NEXT'])
        self.client = robot_client
        self.target_point = target_point

    def execute(self, ud):
        self.client.send_goal(self.target_point)
        return 'NEXT'

class waiting(smach.State):
    def __init__(self, userdata, robot_client, route_index):  # route_index: 0 - левый коридор, 1 - правый коридор
        super().__init__(self, outcomes=['NEXT', 'WAIT'])
        self.client = robot_client
        self.route_index = route_index

    def execute(self, userdata):
        corridor_index = ROBOT_FLAGS[self.client.robot_name][self.route_index]  # находим индекс проверяемого корридора
        if corridor_index == -1:
            if avaliable_corridor[corridor_index]:
                avaliable_corridor[corridor_index] = 0
                return 'NEXT'
        time.sleep(5)
        return 'WAIT'


# class go_into_corridor(smach.State):
#     def __init__(self, robot_client, target_point):
#         super().__init__(outcomes=['GO_THROUGH'])
#         self.client = robot_client
#         self.target_point = target_point

#     def execute(self, ud):
#         self.client.send_goal(self.target_point)
#         return 'GO_THROUGH'




# class go_through_corridor(smach.State):
#     def __init__(self, robot_client, target_point):
#         super().__init__(self, outcomes=['GO_OUT'])
#         self.client = robot_client
#         self.target_point = target_point

#     def execute(self, ud):
        
#         return 'GO_OUT'
        


# class go_out_of_corridor(smach.State):
#     def __init__(self, robot_client, target_point):
#         smach.State.__init__(self, outcomes=['WAIT'])
#         self.client = robot_client
#         self.target_point = target_point

#     def execute(self, ud):
        
#         return 'WAIT'



# class waiting(smach.State):
#     def __init__(self, userdata, robot_client, route_index):  # route_index: 0 - левый коридор, 1 - правый коридор
#         super().__init__(self, outcomes=['GO_TO', 'WAIT'])
#         self.client = robot_client
#         self.route_index = route_index

#     def execute(self, userdata):
#         corridor_index = ROBOT_FLAGS[self.client.robot_name][self.route_index]  # находим индекс проверяемого корридора
#         if corridor_index == -1:
#             if avaliable_corridor[corridor_index]:
#                 avaliable_corridor[corridor_index] = 0
#                 return 'GO_TO'
#         time.sleep(5)
#         return 'WAIT'

# class waiting(smach.State):
#     def __init__(self, userdata, robot_client, route_index):  # route_index: 0 - левый коридор, 1 - правый коридор
#         super().__init__(self, outcomes=[self.generate_way_string(route_index), f'{robot_client.robot_name}_WAIT' ])
#         self.client = robot_client
#         self.route_index = route_index

#     def generate_way_string(self, route_index):
#         way_string = f'{self.client.robot_name}'
#         if not route_index:
#             way_string += '_go_to_left' 
#         else:
#             way_string += '_go_to_right'
#         return way_string

#     def execute(self, userdata):
#         corridor_index = ROBOT_FLAGS[self.client.robot_name][self.route_index]  # находим индекс проверяемого корридора
#         if corridor_index == -1:
#             if avaliable_corridor[corridor_index]:
#                 avaliable_corridor[corridor_index] = 0
#                 return self.generate_way_string(self.route_index)
#         time.sleep(5)
#         return f'{self.client.robot_name}_WAIT'

            



def main(args=None):
    rclpy.init(args=args)
    node = MultiRobotManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()