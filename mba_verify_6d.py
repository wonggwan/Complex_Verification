import random
import matlab.engine
from multi_MBA.task import Task_6D, task_analyzer
from multi_MBA.buchi_parse import Buchi


def main():
    task = Task_6D()
    print("\nOverall Command:\n{}".format(task.formula))
    print("\nCorresponding sub-tasks:\n{}".format(task.subformula))

    # multi-MBA initialization
    buchi = Buchi(task)
    buchi.construct_buchi_graph()
    buchi.get_minimal_length()
    buchi.get_feasible_accepting_state()
    acceptingNBAState = buchi.buchi_graph.graph['accept'][0]
    currentNBAState = buchi.buchi_graph.graph['init'][0]

    """System model setting (ts=0.1)"""
    g = 9.81
    # Continuous A, B matrix
    Ac = matlab.double([[0, 0, 0, 1, 0, 0],
                        [0, 0, 0, 0, 1, 0],
                        [0, 0, 0, 0, 0, 1],
                        [0, 0, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0, 0]])
    Bc = matlab.double([[0, 0, 0],
                        [0, 0, 0],
                        [0, 0, 0],
                        [g, 0, 0],
                        [0, -g, 0],
                        [0, 0, 0]])
    Ec = matlab.double([[0], [0], [0], [0], [0], [-1]])

    # Initial state set
    q0 = matlab.double([[4.7], [4.7], [3.0], [0.95], [0], [0]])
    Q0 = matlab.double([0.05, 0.05, 0.05, 0.01, 0.01, 0.01])

    # Avoiding set and Goal set
    avoid_set_xyz = matlab.double([4.5, 4.7, 4.5, 5, 2.5, 2.6])
    goal_set_xyz = matlab.double([4, 5, 4.3, 5, 2.3, 2.7])

    # room dictionary
    room_goal_dict = {
        '1': matlab.double([4, 5, 4.3, 5, 2.3, 2.7])
    }

    while currentNBAState != acceptingNBAState:
        nextNBAState = buchi.get_next_NBA_state(currentNBAState, acceptingNBAState)
        print("nextNBAState chosen -> ", nextNBAState)
        nextAction = buchi.get_next_action(currentNBAState, nextNBAState)
        print("Next action -> {}".format(nextAction))
        ok_Action = {k: v for (k, v) in nextAction.items() if v is True}
        print("Runnable action -> {}".format(ok_Action))
        sub_task, _ = random.choice(list(ok_Action.items()))
        room_num = task_analyzer(sub_task)



if __name__ == '__main__':
    main()
