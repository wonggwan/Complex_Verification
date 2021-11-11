import random
import matlab.engine
from multi_MBA.task import Task, task_analyzer
from multi_MBA.buchi_parse import Buchi


def main():
    # random.seed(9)
    task = Task()
    print("\nOverall Command:\n{}".format(task.formula))
    print("\nCorresponding sub-tasks:\n{}".format(task.subformula))

    # multi-MBA initialization
    buchi = Buchi(task)
    buchi.construct_buchi_graph()
    buchi.get_minimal_length()
    buchi.get_feasible_accepting_state()
    acceptingNBAState = buchi.buchi_graph.graph['accept'][0]
    currentNBAState = "T0_init"

    # System Dynamics: x_{t+1} = A * x_{t} + B * u_{t}
    A = matlab.double([[-0.5, 0], [0.1, -0.2]])
    B = matlab.double([[1, 0], [0, 1]])

    # avoiding sets
    avoid_x = matlab.double([1, 1.2, 1.2, 1])
    avoid_y = matlab.double([1, 1, 1.2, 1.2])

    # initial states polyhedron
    X_poly = matlab.double([[1, 2], [1.5, 2], [1.5, 2.5], [1, 2.5]])

    # room dictionary
    room_goal_dict = {
        '1': [matlab.double([1.95, 2.05, 2.05, 1.95]),  # reachable
              matlab.double([0.98, 0.98, 1.02, 1.02])],
        # '1': [matlab.double([3.95, 3.05, 4.05, 7.95]),  # not reachable
        #       matlab.double([0.98, 0.98, 1.02, 1.02])],
        # '2': [matlab.double([2.45, 2.55, 2.55, 2.45]),  # reachable
        #       matlab.double([2.1, 2.1, 2.3, 2.3])],
        '2': [matlab.double([3.45, 3.55, 3.55, 3.45]),  # not reachable
              matlab.double([3.1, 3.1, 3.3, 3.3])],
        '3': [matlab.double([1.18, 1.25, 1.25, 1.18]),  # reachable
              matlab.double([1.38, 1.38, 1.42, 1.42])],
        # '3': [matlab.double([3.18, 3.25, 3.25, 3.18]),  # not reachable
        #       matlab.double([1.38, 1.38, 1.42, 1.42])],
        '4': [matlab.double([-1.01, -0.98, -0.98, -1.01]),  # reachable
              matlab.double([-1.01, -1.01, -0.98, -0.98])],
        # '4': [matlab.double([-5.01, -5.98, -5.98, -5.01]),  # not reachable
        #       matlab.double([-1.01, -1.01, -0.98, -0.98])]
    }

    """
    controller 1: [2; 1]
    controller 2: [2.5; 2.2]
    controller 3: [1.2; 1.4]
    controller 4: [-1; -1]
    """

    while currentNBAState != acceptingNBAState:
        nextNBAState = buchi.get_next_NBA_state(currentNBAState, acceptingNBAState)
        nextAction = buchi.get_next_action(currentNBAState, nextNBAState)
        print(nextAction)
        ok_Action = {k: v for (k, v) in nextAction.items() if v is True}
        sub_task, _ = random.choice(list(ok_Action.items()))
        room_num = task_analyzer(sub_task)
        print("\n--------------------------------------")
        print("Current Sub-Task -> {}\n".format(room_num))
        eng = matlab.engine.start_matlab()
        cur_room_goal_x, cur_room_goal_y = room_goal_dict[room_num][0], room_goal_dict[room_num][1]
        print("[Sub-Task {}] Current room goal:\nx -> {}\ny -> {}".format(room_num, cur_room_goal_x, cur_room_goal_y))
        cur_controller = './output/room' + str(room_num) + '.mat'
        result = eng.rsdp(A, B, X_poly, avoid_x, avoid_y, cur_room_goal_x, cur_room_goal_y, cur_controller, nargout=2)
        eng.quit()
        is_satisfied = result[0]
        FRS_V_bd = result[1]
        if is_satisfied:
            print("\nSub-Task {} is satisfied".format(room_num))
            currentNBAState = nextNBAState
            X_poly = FRS_V_bd
        else:
            print("\nSub-Task {} is not satisfied".format(room_num))
            flagAlternate = buchi.update_alternate_transition(currentNBAState, nextNBAState)  # update alternate action
            if not flagAlternate:
                is_verification_failed = buchi.delete_transition(currentNBAState, nextNBAState)
                if is_verification_failed:
                    print("\n[Buchi] No more transitions to accepting state, verification failed.")
                    exit(1)
    print("\nAll sub-tasks for command ({}) are satisfied, verification succeeded.".format(task.formula))


if __name__ == '__main__':
    main()
