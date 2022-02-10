import random
import math
import matlab.engine
import numpy as np
from multi_MBA.task import Task_6D, task_analyzer
from multi_MBA.buchi_parse import Buchi
from scipy.linalg import block_diag


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
    sdp_iter = 8
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
    q0 = [[4.3], [4.3], [4.3], [-0.5], [-0.5], [-0.5]]
    Q0 = [0.05, 0.05, 0.05, 0.01, 0.01, 0.01]

    RERUN_Q = [0.01, 0.01, 0.01, 0.01, 0.01, 0.01]  # reshape the ellipsoid in future ellipsoid if RERUN needed

    # room dictionary
    room_goal_dict = {
        '0': [-0.5, 0.5, -0.5, 0.5, -0.5, 0.5],
        '1': [0.5, 1.5, 0.5, 1.5, 0.5, 1.5],
        '2': [1.5, 2.5, 1.5, 2.5, 1.5, 2.5],
        '3': [2.5, 3.5, 2.5, 3.5, 2.5, 4.0],
        '4': [1.3, 1.4, 1.3, 1.4, 1.3, 1.4],
        '5': [1.1, 1.2, 1.1, 1.2, 1.1, 1.2]
    }

    # Find the avoiding room information
    avoid_set_xyz = []
    process_counter = 0
    always_task = ''
    final_state = list(buchi.buchi_graph.edges)[-1]
    if final_state[0] == final_state[1] and final_state[1] != 'accept_all':
        always_task = buchi.buchi_graph.edges[final_state]['AP']
    if always_task:
        clear_formula = always_task.replace(" ", "").replace("(", "").replace(")", "").split("&&")
        for sub in clear_formula:
            if sub.startswith('!'):
                avoid_set_xyz.append(room_goal_dict[sub[2:]])  # !e[index], we only need index
    avoid_set_xyz = matlab.double(avoid_set_xyz)

    LAST_SUCCESS_q = q0   # The last successful ellipsoid info is defined here
    LAST_SUCCESS_Q = block_diag(Q0[0]**2, Q0[1]**2, Q0[2]**2, Q0[3]**2, Q0[4]**2, Q0[5]**2).tolist()
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

        # RERUN related info
        RERUN_COUNTER = 0
        FINAL_FAIL_JUDGE_FLAG = 0
        RETRY_SUCCESS_FLAG = 0

        print("LAST SUCCESS ->", LAST_SUCCESS_Q)
        q = matlab.double(LAST_SUCCESS_q)
        Q = matlab.double(LAST_SUCCESS_Q)

        print("\n--------------------------------------")
        print("Current Sub-Task -> {}\n".format(room_num))
        eng = matlab.engine.start_matlab()
        goal_set_xyz = matlab.double(room_goal_dict[room_num])
        cur_controller = './output/quad_mpc_' + str(room_num) + '.mat'
        is_satisfied, res_q, res_Q = eng.rsdp_6d(Ac, Bc, Ec, g, ts, q, Q,
                                                 avoid_set_xyz, goal_set_xyz,
                                                 cur_controller, process_counter, sdp_iter, nargout=3)
        eng.quit()
        # Regardless of successful or not, initial state will be updated at next time
        if is_satisfied:
            print("\nSub-Task {} is satisfied".format(room_num))
            currentNBAState = nextNBAState
            LAST_SUCCESS_q = matlab.double(res_q)
            LAST_SUCCESS_Q = matlab.double(res_Q)
        else:
            """
            When a task is verified to be 'not-reachable'
            * For the first time: we use the same center but apply a smaller shape matrix 
            * Rerun the test again
                * If succeeded
                    * Gradually enlarge the shape size until it fails again
                    * Use the last 'reachable' ellipsoid for the final result of this task
                * If failed again
                    * We treat this as 'not-reachable' for real and delete the multi-MBA edge in this case
            """
            print("\nSub-Task {} is not satisfied".format(room_num))
            PREV_q, PREV_Q = LAST_SUCCESS_q, LAST_SUCCESS_Q

            while FINAL_FAIL_JUDGE_FLAG != 1 and RETRY_SUCCESS_FLAG != 1:
                RERUN_Q_DIAG = block_diag(RERUN_Q[0] ** 2, RERUN_Q[1] ** 2, RERUN_Q[2] ** 2,
                                          RERUN_Q[3] ** 2, RERUN_Q[4] ** 2, RERUN_Q[5] ** 2).tolist()
                process_counter += 1
                RERUN_COUNTER += 1
                print("\n\n--------------------------------------")
                print("Retrying Sub-Task {} ====> Try.{}\n".format(room_num, RERUN_COUNTER))
                eng = matlab.engine.start_matlab()
                goal_set_xyz = matlab.double(room_goal_dict[room_num])
                cur_controller = './output/quad_mpc_' + str(room_num) + '.mat'

                q = matlab.double(LAST_SUCCESS_q)  # rerun always use the previous successful ellipsoid center
                Q = matlab.double(RERUN_Q_DIAG)  # When re-run, shape matrix Q is updated
                is_satisfied, res_q, res_Q = eng.rsdp_6d(Ac, Bc, Ec, g, ts, q, Q,
                                                         avoid_set_xyz, goal_set_xyz,
                                                         cur_controller, process_counter, sdp_iter, nargout=3)
                eng.quit()
                if not is_satisfied and RERUN_COUNTER == 1:
                    # Failed right at the 1st time after shrinking ellipsoid, then say it's failed
                    # Use the last successful ellipsoid when jump to next task
                    FINAL_FAIL_JUDGE_FLAG = 1
                    flagAlternate = buchi.update_alternate_transition(currentNBAState, nextNBAState)
                    if not flagAlternate:
                        is_verification_failed = buchi.delete_transition(currentNBAState, nextNBAState)
                        if is_verification_failed:
                            print("\n[Buchi] No more transitions to accepting state, verification failed.")
                            exit(1)
                if is_satisfied:
                    # Succeeded only after the 1st time the ellipsoid is re-applied
                    print("\nSub-Task {} is satisfied for the {} retry".format(room_num, RERUN_COUNTER))
                    print("Enlarge shape of initial ellipsoid and rerun test")
                    PREV_q = res_q
                    PREV_Q = res_Q
                    success_Q = np.array(LAST_SUCCESS_Q)
                    print(success_Q)
                    x = RERUN_Q[0] + abs(math.sqrt(success_Q[0, 0])) / 4
                    y = RERUN_Q[1] + abs(math.sqrt(success_Q[1, 1])) / 4
                    z = RERUN_Q[2] + abs(math.sqrt(success_Q[2, 2])) / 4
                    RERUN_Q = [x, y, z, 0.01, 0.01, 0.01]
                if not is_satisfied and RERUN_COUNTER != 1:
                    # Survived some rerun test
                    # Failed at some future step
                    # Use the previous succeeded one
                    RETRY_SUCCESS_FLAG = 1
                    print("\nSub-Task {} is satisfied after retry {}".format(room_num, RERUN_COUNTER))
                    currentNBAState = nextNBAState
                    LAST_SUCCESS_q = matlab.double(PREV_q)
                    LAST_SUCCESS_Q = matlab.double(PREV_Q)
                    print("Current Last Success q", LAST_SUCCESS_q)


if __name__ == '__main__':
    main()
