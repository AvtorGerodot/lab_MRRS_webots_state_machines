#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
import smach


class Start(smach.State):
    def __init__(self):
        super().__init__(outcomes=['to_sub'])

    def execute(self, userdata):
        print("START (top machine)")
        time.sleep(1)
        return 'to_sub'


class End(smach.State):
    def __init__(self):
        super().__init__(outcomes=['finished'])

    def execute(self, userdata):
        print("END (top machine)")
        time.sleep(1)
        return 'finished'

class Work1(smach.State):
    def __init__(self):
        super().__init__(outcomes=['next'])

    def execute(self, userdata):
        print("SubMachine: WORK1")
        time.sleep(1)
        return 'next'


class Work2(smach.State):
    def __init__(self):
        super().__init__(outcomes=['done'])

    def execute(self, userdata):
        print("SubMachine: WORK2")
        time.sleep(1)
        return 'done'


def main(args=None):
    rclpy.init(args=args)

    sub_sm = smach.StateMachine(outcomes=['sub_done'])
    with sub_sm:
        smach.StateMachine.add('WORK1', Work1(),
                               transitions={'next': 'WORK2'})
        smach.StateMachine.add('WORK2', Work2(),
                               transitions={'done': 'sub_done'})

    top_sm = smach.StateMachine(outcomes=['finished'])
    with top_sm:
        smach.StateMachine.add('START', Start(),
                               transitions={'to_sub': 'SUBTASK'})

        smach.StateMachine.add('SUBTASK', sub_sm,
                               transitions={'sub_done': 'END'})

        smach.StateMachine.add('END', End(),
                               transitions={'finished': 'finished'})

    outcome = top_sm.execute()
    print(f"\nThe top machine has finished with an outcome: {outcome}")

    rclpy.shutdown()


if __name__ == '__main__':
    main()

