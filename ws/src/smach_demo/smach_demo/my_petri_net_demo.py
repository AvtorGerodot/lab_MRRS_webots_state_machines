#!/usr/bin/env python3
import rclpy
import time
import smach
import numpy as np


class Rb1_P1(smach.State):
    def __init__(self):
        super().__init__(outcomes=['t1'])

    def execute(self, userdata):
        print("Rb1: p1 (вне)")
        time.sleep(1)
        return 't1'


class Rb1_P5(smach.State):
    def __init__(self):
        super().__init__(outcomes=['t7'])

    def execute(self, userdata):
        print("Rb1: p5 (на общей линии с роботом 2)")
        time.sleep(1)
        return 't7'


class Rb2_P2(smach.State):
    def __init__(self):
        super().__init__(outcomes=['t2', 't3'])

    def execute(self, userdata):
        print(">>> Выбор направления у робота 2 в P2")
        time.sleep(1)
        
        if np.random.random() > 0.5:
            return 't2'
        else:
            return 't3'


class Rb2_P6(smach.State):
    def __init__(self):
        super().__init__(outcomes=['t8'])

    def execute(self, userdata):
        print("Rb2: p6 (на общей линии с роботом 1)")
        time.sleep(1)
        return 't8'


class Rb2_P7(smach.State):
    def __init__(self):
        super().__init__(outcomes=['t9'])

    def execute(self, userdata):
        print("Rb2: p7 (на общей линии с роботом 3)")
        time.sleep(1)
        return 't9'


class Rb3_P3(smach.State):
    def __init__(self):
        super().__init__(outcomes=['t4', 't5'])

    def execute(self, userdata):
        print(">>> Выбор направления у робота 3 в P3")
        time.sleep(1)
        
        if np.random.random() > 0.5:
            return 't4'
        else:
            return 't5'
        

class Rb3_P8(smach.State):
    def __init__(self):
        super().__init__(outcomes=['t10'])

    def execute(self, userdata):
        print("Rb3: p8 (на общей линии с роботом 2)")
        time.sleep(1)
        return 't10'
    

class Rb3_P9(smach.State):
    def __init__(self):
        super().__init__(outcomes=['t11'])

    def execute(self, userdata):
        print("Rb3: p9 (на общей линии с роботом 4)")
        time.sleep(1)
        return 't11'
         

class Rb4_P4(smach.State):
    def __init__(self):
        super().__init__(outcomes=['t6'])

    def execute(self, userdata):
        print("Rb4: p4 (вне)")
        time.sleep(1)
        return 't6'


class Rb4_P10(smach.State):
    def __init__(self):
        super().__init__(outcomes=['t12'])

    def execute(self, userdata):
        print("Rb4: p10 (на общей линии с роботом 3)")
        time.sleep(1)
        return 't12'
    

class P12(smach.State):
    def __init__(self):
        super().__init__(outcomes=['to_rb1', 'to_rb2', 'end'])
        self.counter = 0

    def execute(self, userdata):
        print(">>>  Определение приоритета 1 или 2 в p12")
        time.sleep(1)
        if self.counter % 3 == 0:
            self.counter += 1
            return 'to_rb1'
        elif self.counter < 10:
            self.counter += 1
            return 'to_rb2'
        else:
            return 'end'


class P23(smach.State):
    def __init__(self):
        super().__init__(outcomes=['to_rb3', 'to_rb4'])
        self.counter = 0

    def execute(self, userdata):
        print(">>> Определение приоритета 2 или 3 в p23")
        time.sleep(1)
        if self.counter % 2 == 0:
            self.counter += 1
            return 'to_rb3'
        else:
            return 'to_rb4'
        

class P34(smach.State):
    def __init__(self):
        super().__init__(outcomes=['to_rb5', 'to_rb6'])
        self.counter = 0

    def execute(self, userdata):
        print(">>> Определение приоритета 3 или 4 в p34")
        time.sleep(1)
        if self.counter % 3 == 0:
            self.counter += 1
            return 'to_rb6'
        else:
            return 'to_rb5'
        

def main(args=None):
    rclpy.init(args=args)

    sm_rb1 = smach.StateMachine(outcomes=['to_p12'])
    with sm_rb1:
        smach.StateMachine.add('P1', Rb1_P1(),
                               transitions={'t1': 'P5'})
        smach.StateMachine.add('P5', Rb1_P5(),
                               transitions={'t7': 'to_p12'})

    sm_rb2 = smach.StateMachine(outcomes=['to_p12', 'to_p23'])
    with sm_rb2:
        smach.StateMachine.add('P2', Rb2_P2(),
                               transitions={'t2': 'P6',
                                            't3':'P7' })
        smach.StateMachine.add('P6', Rb2_P6(),
                               transitions={'t8': 'to_p12'})
        smach.StateMachine.add('P7', Rb2_P7(),
                               transitions={'t9': 'to_p23'})
    
    sm_rb3 = smach.StateMachine(outcomes=['to_p34', 'to_p23'])
    with sm_rb2:
        smach.StateMachine.add('P3', Rb3_P3(),
                               transitions={'t4': 'P8',
                                            't5':'P9' })
        smach.StateMachine.add('P8', Rb3_P8(),
                               transitions={'t10': 'to_p23'})
        smach.StateMachine.add('P9', Rb3_P9(),
                               transitions={'t11': 'to_p34'})
    
    sm_rb4 = smach.StateMachine(outcomes=['to_p34'])
    with sm_rb1:
        smach.StateMachine.add('P4', Rb4_P4(),
                               transitions={'t6': 'P10'})
        smach.StateMachine.add('P10', Rb4_P10(),
                               transitions={'t12': 'to_p34'})


    sm_top = smach.StateMachine(outcomes=['finished'])
    with sm_top:
        smach.StateMachine.add('P12', P12(),
                               transitions={'to_rb1': 'RB1',
                                            'to_rb2': 'RB2',
                                            'end': 'finished'})
        
        smach.StateMachine.add('P23', P23(),
                               transitions={'to_rb3': 'RB3',
                                            'to_rb2': 'RB2'})
        
        smach.StateMachine.add('P34', P34(),
                               transitions={'to_rb3': 'RB3',
                                            'to_rb4': 'RB4'})

        smach.StateMachine.add('RB1', sm_rb1,
                               transitions={'to_p12': 'P12'})
        smach.StateMachine.add('RB2', sm_rb2,
                               transitions={'to_p12': 'P12',
                                            'to_p23': 'P23'})
        smach.StateMachine.add('RB3', sm_rb3,
                               transitions={'to_p34': 'P34',
                                            'to_p23': 'P23'})
        smach.StateMachine.add('RB4', sm_rb4,
                               transitions={'to_p34': 'P34'})

    outcome = sm_top.execute()
    print(f"\nМашина завершила работу с исходом: {outcome}")

    rclpy.shutdown()


if __name__ == '__main__':
    main()
