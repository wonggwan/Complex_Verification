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
        # self.formula = '<>e1 && <>e2 && <>e3'
        # self.formula = '<>(e3 && <>(e2 && (<>e1))))'
        # self.formula = '<>(e1 && <>(e3 && (<>e2))))'
        # self.formula = '<>(e3 && <>(e2 && (<>e1))))'
        # self.formula = '<>(e3 && <>(e2 && (<>[]e1)))) && []!e5'
        # self.formula = '<>(e3 && <>[]e1)'
        # self.formula = '<>[]e1'
        self.formula = '<>e1 && <>e2 && []!e4 && []!e5'
        # self.formula = '[]!e5 && <>e1 && <>[]e2'
        self.subformula = {
            0: ['(l0_1)', 1, 3],
            1: ['(l1_1)', 1, 3],
            2: ['(l2_1)', 1, 3],
            3: ['(l3_1)', 1, 3],
            4: ['(l4_1)', 1, 3],
            5: ['(l5_1)', 1, 3]
        }
        self.number_of_robots = 1


def task_analyzer(task):
    room_num = task.split('l')[1].split('_')[0]
    return room_num
