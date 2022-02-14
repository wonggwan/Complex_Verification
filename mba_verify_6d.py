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
    sdp_iter = 9
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
    q0 = [[3.1], [3.1], [3.1], [-0.5], [-0.5], [-0.5]]
    Q0 = [0.05, 0.05, 0.05, 0.01, 0.01, 0.01]

    # room dictionary
    room_goal_dict = {
        '0': [-0.5, 0.5, -0.5, 0.5, -0.5, 0.5],
        '1': [0.5, 1.5, 0.5, 1.5, 0.5, 1.5],
        '2': [1.5, 3.0, 1.5, 3.0, 1.5, 2.7],
        '3': [3.0, 3.7, 3.0, 3.5, 2.7, 4.0],
        '4': [-1.3, -1.4, -1.3, -1.4, -1.3, -1.4],
        '5': [-1.1, -1.2, -1.1, -1.2, -1.1, -1.2],
        '6': [3.5, 4.5, 3.5, 4.5, -0.5, 0.5]
    }

    # Find the avoiding room information
    avoid_set_xyz = []
    process_counter = 0
    final_state = list(buchi.buchi_graph.edges)[-1]
    if final_state[0] == final_state[1] and final_state[1] != 'accept_all':
        if type(buchi.buchi_graph.edges[final_state]['truth']) is dict:
            for element in list(buchi.buchi_graph.edges[final_state]['truth'].keys()):  # Find only []! type command
                avoid_set_xyz.append(room_goal_dict[element[1]])
    avoid_set_xyz = matlab.double(avoid_set_xyz)

    LAST_SUCCESS_q = q0  # The last successful ellipsoid info is defined here
    LAST_SUCCESS_Q = block_diag(Q0[0] ** 2, Q0[1] ** 2, Q0[2] ** 2, Q0[3] ** 2, Q0[4] ** 2, Q0[5] ** 2).tolist()
    while currentNBAState != acceptingNBAState:
        process_counter += 1
        nextNBAState = buchi.get_next_NBA_state(currentNBAState, acceptingNBAState)
        print("nextNBAState chosen -> ", nextNBAState)
        nextAction = buchi.get_next_action(currentNBAState, nextNBAState)
        print("Next action -> {}".format(nextAction))

        robot_set, ok_Action = set(), []
        for key, values in nextAction.items():
            robot_set.add(key[-1])
            if values:
                ok_Action.append(key)
        print("Runnable action -> {}".format(ok_Action))
        if len(robot_set) > 1:
            print('multiple robots, needs to arrive simultaneously')
        sub_task = random.choice(ok_Action)
        room_num = task_analyzer(sub_task)

        # RERUN related info
        RERUN_COUNTER = 0
        FINAL_FAIL_JUDGE_FLAG = 0
        RETRY_SUCCESS_FLAG = 0

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
            SPEED_SUB_MATRIX = np.array(LAST_SUCCESS_Q)[3:6, 3:6]  # RE-USE SPEED FROM PREVIOUS TASK
            RERUN_Q_TMP = [0.01, 0.01, 0.01]
            optimal_ratio = 0

            while FINAL_FAIL_JUDGE_FLAG != 1 and RETRY_SUCCESS_FLAG != 1:
                print("\n\n--------------------------------------")
                process_counter += 1
                RERUN_COUNTER += 1
                print("Retrying Sub-Task {} ====> Try No.{}\n".format(room_num, RERUN_COUNTER))

                RERUN_Q_DIAG = block_diag(RERUN_Q_TMP[0] ** 2, RERUN_Q_TMP[1] ** 2,
                                          RERUN_Q_TMP[2] ** 2, SPEED_SUB_MATRIX).tolist()
                new_volume = np.sqrt(
                    np.linalg.det(np.array(block_diag(RERUN_Q_TMP[0] ** 2, RERUN_Q_TMP[1] ** 2, RERUN_Q_TMP[2] ** 2))))
                old_volume = np.sqrt(np.linalg.det(np.array(LAST_SUCCESS_Q)[0:3, 0:3]))
                volume_ratio = new_volume / old_volume

                print("Current ellipsoid volume ratio is: {}".format(volume_ratio))

                print("Current denominator size: {}".format(2))

                if volume_ratio >= 1:
                    print(RERUN_Q_TMP)
                    print("\nVolume ratio >= 1")
                    print("\nCurrent retry ended, use result from previous iteration")
                    process_counter -= 1
                    print("\n Process_counter will be switched back to {}".format(process_counter))
                    print("\nSub-Task {} is satisfied after retry {}".format(room_num, RERUN_COUNTER - 1))
                    currentNBAState = nextNBAState
                    LAST_SUCCESS_q = matlab.double(PREV_q)
                    LAST_SUCCESS_Q = matlab.double(PREV_Q)
                    print("Optimal ratio is {}".format(optimal_ratio))
                    break

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
                    print("\nSub-Task {} is satisfied for the No.{} retry".format(room_num, RERUN_COUNTER))
                    print("Enlarge shape of initial ellipsoid and rerun test")
                    PREV_q = res_q
                    PREV_Q = res_Q
                    success_Q = np.array(LAST_SUCCESS_Q)
                    # Only change the volume of the retry-initial-matrix
                    RERUN_Q_TMP[0] += abs(math.sqrt(success_Q[0, 0]) - RERUN_Q_TMP[0]) / 2
                    RERUN_Q_TMP[1] += abs(math.sqrt(success_Q[1, 1]) - RERUN_Q_TMP[1]) / 2
                    RERUN_Q_TMP[2] += abs(math.sqrt(success_Q[2, 2]) - RERUN_Q_TMP[2]) / 2

                    optimal_ratio = volume_ratio

                if not is_satisfied and RERUN_COUNTER != 1:
                    # Survived some rerun test
                    # Failed at some future step
                    # Use the previous succeeded one
                    RETRY_SUCCESS_FLAG = 1
                    print("\nCurrent retry failed, use result from previous iteration")
                    print("\nSub-Task {} is satisfied after retry {}".format(room_num, RERUN_COUNTER-1))
                    currentNBAState = nextNBAState
                    LAST_SUCCESS_q = matlab.double(PREV_q)
                    LAST_SUCCESS_Q = matlab.double(PREV_Q)
                    print("Optimal ratio is {}".format(optimal_ratio))


if __name__ == '__main__':
    main()
