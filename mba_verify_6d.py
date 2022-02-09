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

    """System model setting"""
    g = 9.81
    ts = 0.3
    sdp_iter = 2
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
                        [0, 0, 1]])
    Ec = matlab.double([[0], [0], [0], [0], [0], [-1]])

    # Initial state set
    q0 = matlab.double([[4.1], [4.44], [4.4], [-0.5], [-0.5], [-0.5]])
    # q0 = matlab.double([[2.1], [2.44], [2.4], [-0.5], [-0.5], [-0.5]])

    Q0 = matlab.double([0.05, 0.05, 0.05, 0.01, 0.01, 0.01])
    is_init = True  # The initial state set is only needed at the first time

    # Avoiding set and Goal set
    avoid_set_xyz = matlab.double([1.1, 1.2, 1.1, 1.2, 1.1, 1.2])  # avoid set

    # room dictionary
    room_goal_dict = {
        '1': matlab.double([0.5, 1.5, 0.5, 1.5, 0.5, 1.5]),
        '2': matlab.double([1.5, 2.5, 1.5, 2.5, 1.5, 2.5]),
        '3': matlab.double([2.5, 3.5, 2.5, 3.5, 2.5, 4.0])
    }

    process_counter = 0
    edge_list = []
    for e in buchi.buchi_graph.edges:
        edge_list.append(e)
        print(e, buchi.buchi_graph.edges[e])
    print(edge_list)
    final_state = edge_list[-1]
    if final_state[0] == final_state[1] and final_state[1] != 'accept_all':
        print(type(buchi.buchi_graph.edges[final_state]['AP']))
        print("This command includes '[]' ")

    while currentNBAState != acceptingNBAState:
        process_counter += 1
        nextNBAState = buchi.get_next_NBA_state(currentNBAState, acceptingNBAState)
        print("nextNBAState chosen -> ", nextNBAState)
        nextAction = buchi.get_next_action(currentNBAState, nextNBAState)
        print("Next action -> {}".format(nextAction))
        ok_Action = {k: v for (k, v) in nextAction.items() if v is True}
        print("Runnable action -> {}".format(ok_Action))
        sub_task, _ = random.choice(list(ok_Action.items()))
        room_num = task_analyzer(sub_task)

        # need a "is_init" flag to take different ellispoid
        # multiple controller? or 1 controller with multiple objectives?

        print("\n--------------------------------------")
        print("Current Sub-Task -> {}\n".format(room_num))
        eng = matlab.engine.start_matlab()
        goal_set_xyz = room_goal_dict[room_num]
        cur_controller = './output/quad_mpc_' + str(room_num) + '.mat'
        result = eng.rsdp_6d(Ac, Bc, Ec, g, ts, q0, Q0,
                             avoid_set_xyz, goal_set_xyz,
                             cur_controller, is_init, process_counter, sdp_iter, nargout=1)
        eng.quit()
        is_init = False  # q0, Q0 and is_init will not be useful after 1st iteration
        is_satisfied = result
        if is_satisfied:
            print("\nSub-Task {} is satisfied".format(room_num))
            currentNBAState = nextNBAState
        else:
            print("\nSub-Task {} is not satisfied".format(room_num))
            flagAlternate = buchi.update_alternate_transition(currentNBAState, nextNBAState)  # update alternate action
            if not flagAlternate:
                is_verification_failed = buchi.delete_transition(currentNBAState, nextNBAState)
                if is_verification_failed:
                    print("\n[Buchi] No more transitions to accepting state, verification failed.")
                    exit(1)


if __name__ == '__main__':
    main()
