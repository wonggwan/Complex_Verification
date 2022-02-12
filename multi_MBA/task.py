# Define verification task
class Task(object):
    def __init__(self):
        # apply a realistic case study (high-priority)
        # use some real linear system case modeled for a specific robot
        # maybe non-linear system as well
        # approximate non-linear system with linear system
        # debug nextAction case

        self.formula = '<>e1 && <>(e2 && <>e3) && <>e4 && []!e5'
        # self.formula = '<>e1 && <>e2 && <>e3 && <>e4 && []!e5'
        # self.formula = '<>e1 && <>(e2 || e3) && <>e4 && []!e5'
        # self.formula = '<>e1 && <>(e2 && <>(e3 && <>e4)) && []!e5'
        self.subformula = {
            1: ['(l1_1)', 1, 3],
            2: ['(l2_1)', 1, 3],
            3: ['(l3_1)', 1, 3],
            4: ['(l4_1)', 1, 3],
            5: ['(l5_1)', 1, 3]
        }
        self.number_of_robots = 1


class Task_6D(object):
    def __init__(self):
        # self.formula = '<>e2 && <>e3 && []!e4 && []!e5'
        # self.formula = '<>e2 && <>e6 && (!e2 U e6) && []!e4 && []!e5'
        self.formula = '<>e2 && <>e3 && (!e2 U e3) && []!e5'
        # verify for each robot
        # robot move at the same time
        # every robot has to go to room at the same time (how many steps)
        # number of successful steps must be the same
        #   -> iteration that reach-sdp need to take to the goal room
        # self.formula = '<>e1 && <>e2 && (!e1 U e2) && []!e5'
        # self.formula = '<> (e2 && e3)'
        # Next action -> {'l3_1': True, 'l2_2': True, 'l5_1': False}
        #  -> inspect next action to see which robot to verify
        #  -> if next action have multiple robots, then we will need to verify things at the same time
        # make the obstacles play a role
        self.subformula = {
            0: ['(l0_1)', 1, 3],
            1: ['(l1_1)', 1, 3],
            2: ['(l2_2)', 1, 3],
            3: ['(l3_1)', 1, 3],
            4: ['(l4_1)', 1, 3],
            5: ['(l5_1)', 1, 3],  # obstacles for robot 1 only
            6: ['(l5_2)', 1, 3]
        }
        self.number_of_robots = 2


def task_analyzer(task):
    room_num = task.split('l')[1].split('_')[0]
    return room_num
