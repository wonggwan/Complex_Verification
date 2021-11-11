# Define verification task
class Task(object):
    def __init__(self):
        # self.formula = '<>e1 && <>(e2 && <>e3) && <>e4 && []!e5'
        # self.formula = '<>e1 && <>e2 && <>e3 && <>e4 && []!e5'
        # self.formula = '<>e1 && <>(e2 || e3) && <>e4 && []!e5'
        self.formula = '<>e1 && <>(e2 && <>(e3 && <>e4)) && []!e5'
        self.subformula = {
            1: ['(l1_1)', 1, 0],
            2: ['(l2_1)', 1, 0],
            3: ['(l3_1)', 1, 0],
            4: ['(l4_1)', 1, 0],
            5: ['(l5_1)', 1, 0]
        }
        # self.formula = '<>e1 && <>e2 []!e5'
        # self.subformula = {
        #     1: ['(l1_1)', 1, 3],
        #     2: ['(l2_1)', 1, 3],
        #     5: ['(l5_1)', 1, 3]
        # }
        self.number_of_robots = 1


def task_analyzer(task):
    room_num = task.split('l')[1].split('_')[0]
    # robot = task.split('_')[1]
    return room_num
