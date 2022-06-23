import time
import math
import matlab.engine
import numpy as np
from multi_MBA.task import Task_6D, multi_robot_task_analyzer
from multi_MBA.buchi_parse import Buchi
from scipy.linalg import block_diag
from util.cosafe_setup import cosafe_obstacles, update_obstacles_rt


def main():
    task = Task_6D()
    print("\nOverall Command: {}".format(task.formula))
    print("Total {} robots involved in this command\n".format(task.number_of_robots))

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
    sdp_iter = 4
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

    # q0 = [[0.4], [0.4], [0.3], [0.5], [0.5], [0.5]]
    # Q0 = [0.05, 0.05, 0.05, 0.01, 0.01, 0.01]
    q0 = [[3.5], [3.5], [2.8], [-0.5], [-0.5], [-0.5]]
    Q0 = [0.05, 0.05, 0.05, 0.01, 0.01, 0.01]

    # room dictionary
    room_goal_dict = {
        '0': [-0.5, 0.5, -0.5, 0.5, -0.5, 0.5],
        '1': [0.5, 1.5, 0.5, 1.5, 0.5, 1.5],
        '2': [1.5, 2.5, 1.5, 2.5, 1.5, 2.5],
        '3': [2.5, 3.5, 2.5, 3.5, 2.5, 4.0],
        '4': [1.5, 1.7, 0.9, 1.1, 0, 1.8],
        '5': [1.6, 1.8, 1.9, 2.0, 0, 1.8],
        '6': [1.5, 2.7, 2.5, 4, 1.5, 2.5]  # For [2,3,2]
    }

    process_counter = 0

    # Find the avoiding room information
    # avoid_dict = {}
    # avoid_room_name = {}
    # final_state = list(buchi.buchi_graph.edges)[-1]
    # if final_state[0] == final_state[1] and final_state[1] != 'accept_all':
    #     if type(buchi.buchi_graph.edges[final_state]['truth']) is dict \
    #             and buchi.buchi_graph.edges[final_state]['truth'] is not None:
    #         for element in list(buchi.buchi_graph.edges[final_state]['truth'].keys()):  # Find only []! type command
    #             if element.split('_')[1] not in avoid_dict:
    #                 avoid_dict[element.split('_')[1]] = []
    #                 avoid_room_name[element.split('_')[1]] = []
    #             avoid_dict[element.split('_')[1]].append(room_goal_dict[element.split('l')[1].split('_')[0]])
    #             avoid_room_name[element.split('_')[1]].append(element.split('l')[1].split('_')[0])

    avoid_dict_, avoid_room_name_ = cosafe_obstacles(task.formula, room_goal_dict)

    print("avoid_dict-> ", avoid_dict_)
    print("avoid_room_name -> ", avoid_room_name_)

    # ROBOT INITIALIZATION POSITION DICTIONARY
    robot_pos_dict = {}
    robot_pos_dict_tmp = {}
    check_state = list(buchi.buchi_graph.edges)
    for state in check_state:
        if isinstance(buchi.buchi_graph.edges[state]['truth'], str):
            continue
        for key, value in buchi.buchi_graph.edges[state]['truth'].items():
            if value:
                if key.split('_')[1] not in robot_pos_dict:
                    robot_pos_dict[key.split('_')[1]] = {}
                    robot_pos_dict_tmp[key.split('_')[1]] = {}
    for key in robot_pos_dict.keys():
        robot_pos_dict[key]['LAST_SUCCESS_q'] = q0
        robot_pos_dict[key]['LAST_SUCCESS_Q'] = block_diag(Q0[0] ** 2, Q0[1] ** 2, Q0[2] ** 2,
                                                           Q0[3] ** 2, Q0[4] ** 2, Q0[5] ** 2).tolist()
        robot_pos_dict_tmp[key]['LAST_SUCCESS_q'] = q0
        robot_pos_dict_tmp[key]['LAST_SUCCESS_Q'] = block_diag(Q0[0] ** 2, Q0[1] ** 2, Q0[2] ** 2,
                                                               Q0[3] ** 2, Q0[4] ** 2, Q0[5] ** 2).tolist()
    start_total = time.time()
    cur_suc = -1
    while currentNBAState != acceptingNBAState:
        nextNBAState = buchi.get_next_NBA_state(currentNBAState, acceptingNBAState)
        print("nextNBAState chosen -> ", nextNBAState)
        nextAction = buchi.get_next_action(currentNBAState, nextNBAState)
        print("Next action -> {}".format(nextAction))

        robot_set, ok_Action = set(), []
        for key, values in nextAction.items():
            if values:
                robot_set.add(key[-1])
                ok_Action.append(key)
        print("Runnable action -> {}".format(ok_Action))
        if len(robot_set) > 1:
            print('Multi-robots-task detected, needs to arrive simultaneously')

        RUN_TILL_SUCCESS_SET = set()
        sub_task_robot_num_set = set()

        for sub_task in ok_Action:
            robot_num, room_num = multi_robot_task_analyzer(sub_task)
            sub_task_robot_num_set.add(robot_num)
            print("\n---------- <Robot {}> ----------".format(robot_num))
            process_counter += 1
            avoid_dict, avoid_room_name = update_obstacles_rt(avoid_dict_, avoid_room_name_,
                                                              room_goal_dict, cur_suc, room_num)

            # avoid_set_xyz = []
            # if robot_num in avoid_dict:  # If avoid room is specified for this robot
            #     avoid_set_xyz = matlab.double(avoid_dict[robot_num])
            # print(avoid_set_xyz)
            # if robot_num in avoid_room_name:
            #     print("Obstacles {} detected for robot {}".format(avoid_room_name[robot_num], robot_num))
            avoid_set_xyz = matlab.double(avoid_dict[room_num])
            print("Obstacles {} detected: {}".format(avoid_room_name_[room_num], avoid_dict[room_num]))

            RERUN_COUNTER = 0
            FINAL_FAIL_JUDGE_FLAG = 0
            RETRY_SUCCESS_FLAG = 0

            LAST_SUCCESS_q = robot_pos_dict[robot_num]['LAST_SUCCESS_q']
            LAST_SUCCESS_Q = robot_pos_dict[robot_num]['LAST_SUCCESS_Q']

            q = matlab.double(LAST_SUCCESS_q)
            Q = matlab.double(LAST_SUCCESS_Q)
            eng = matlab.engine.start_matlab()
            print("Current goal room -> {}".format(room_num))

            # ### Temporary script
            # if room_num == '1':
            #     # print(room_goal_dict['1'])
            #     tmp = []
            #     tmp.append(room_goal_dict['2'])
            #     avoid_set_xyz = matlab.double(tmp)
            #     del tmp

            print("avoid: ", avoid_set_xyz)
            goal_set_xyz = matlab.double(room_goal_dict[room_num])
            cur_controller = './output/quad_mpc_' + str(room_num) + '.mat'
            print("Controller {} applied".format(cur_controller))
            print("Goal room info: {}\n".format(goal_set_xyz))
            is_satisfied, res_q, res_Q, success_iter = eng.rsdp_6d(Ac, Bc, Ec, g, ts, q, Q,
                                                                   avoid_set_xyz, goal_set_xyz,
                                                                   cur_controller, process_counter, sdp_iter, nargout=4)
            eng.quit()
            # Regardless of successful or not, initial state will be updated at next time
            if is_satisfied:
                print("\nSub-Task {} is satisfied".format(room_num))
                # currentNBAState = nextNBAState
                robot_pos_dict_tmp[robot_num]['LAST_SUCCESS_q'] = matlab.double(res_q)
                robot_pos_dict_tmp[robot_num]['LAST_SUCCESS_Q'] = matlab.double(res_Q)
                RUN_TILL_SUCCESS_SET.add(success_iter)
                continue
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
                PREV_TILL_SUCCESS = -1
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
                        np.linalg.det(
                            np.array(block_diag(RERUN_Q_TMP[0] ** 2, RERUN_Q_TMP[1] ** 2, RERUN_Q_TMP[2] ** 2))))
                    old_volume = np.sqrt(np.linalg.det(np.array(LAST_SUCCESS_Q)[0:3, 0:3]))
                    volume_ratio = new_volume / old_volume

                    print("Current ellipsoid volume ratio is: {}".format(volume_ratio))
                    if volume_ratio >= 1:
                        print(RERUN_Q_TMP)
                        print("\nVolume ratio >= 1")
                        print("\nCurrent retry ended, use result from previous iteration")
                        process_counter -= 1
                        print("\nProcess_counter will be switched back to {}".format(process_counter))
                        print("\nSub-Task {} is satisfied after retry {}".format(room_num, RERUN_COUNTER - 1))
                        # currentNBAState = nextNBAState
                        robot_pos_dict_tmp[robot_num]['LAST_SUCCESS_q'] = matlab.double(PREV_q)
                        robot_pos_dict_tmp[robot_num]['LAST_SUCCESS_Q'] = matlab.double(PREV_Q)
                        print("Optimal ratio is {}".format(optimal_ratio))
                        RUN_TILL_SUCCESS_SET.add(PREV_TILL_SUCCESS)
                        break

                    eng = matlab.engine.start_matlab()
                    goal_set_xyz = matlab.double(room_goal_dict[room_num])
                    cur_controller = './output/quad_mpc_' + str(room_num) + '.mat'
                    print("Controller {} applied.".format(cur_controller))
                    print("Goal room info: {}\n".format(goal_set_xyz))

                    q = matlab.double(LAST_SUCCESS_q)  # rerun always use the previous successful ellipsoid center
                    Q = matlab.double(RERUN_Q_DIAG)  # When re-run, shape matrix Q is updated
                    is_satisfied, res_q, res_Q, success_iter = eng.rsdp_6d(Ac, Bc, Ec, g, ts, q, Q,
                                                                           avoid_set_xyz, goal_set_xyz,
                                                                           cur_controller, process_counter, sdp_iter,
                                                                           nargout=4)
                    eng.quit()
                    if not is_satisfied and RERUN_COUNTER == 1:
                        # Failed right at the 1st time after shrinking ellipsoid, then say it's failed
                        # Use the last successful ellipsoid when jump to next task
                        FINAL_FAIL_JUDGE_FLAG = 1
                        RUN_TILL_SUCCESS_SET.add(PREV_TILL_SUCCESS)
                        # flagAlternate = buchi.update_alternate_transition(currentNBAState, nextNBAState)
                        # if not flagAlternate:
                        #     is_verification_failed = buchi.delete_transition(currentNBAState, nextNBAState)
                        #     if is_verification_failed:
                        #         print("\n[Buchi] No more transitions to accepting state, verification failed.")
                        #         exit(1)
                    if is_satisfied:
                        # Succeeded only after the 1st time the ellipsoid is re-applied
                        print("\nSub-Task {} is satisfied for the No.{} retry".format(room_num, RERUN_COUNTER))
                        print("Enlarge shape of initial ellipsoid and rerun test")
                        PREV_TILL_SUCCESS = success_iter
                        RUN_TILL_SUCCESS_SET.add(PREV_TILL_SUCCESS)
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
                        RUN_TILL_SUCCESS_SET.add(PREV_TILL_SUCCESS)
                        print("\nCurrent retry failed, use result from previous iteration")
                        print("\nSub-Task {} is satisfied after retry {}".format(room_num, RERUN_COUNTER - 1))
                        # currentNBAState = nextNBAState
                        robot_pos_dict_tmp[robot_num]['LAST_SUCCESS_q'] = matlab.double(PREV_q)
                        robot_pos_dict_tmp[robot_num]['LAST_SUCCESS_Q'] = matlab.double(PREV_Q)
                        print("Optimal ratio is {}".format(optimal_ratio))
        # Check if the succeeded iteration matches or not
        RUN_TILL_SUCCESS_SET = list(RUN_TILL_SUCCESS_SET)

        if -1 in RUN_TILL_SUCCESS_SET:
            flagAlternate = buchi.update_alternate_transition(currentNBAState, nextNBAState)
            if not flagAlternate:
                is_verification_failed = buchi.delete_transition(currentNBAState, nextNBAState)
                if is_verification_failed:
                    print("\n[Buchi] No more transitions to accepting state, verification failed.")
                    exit(1)
        else:
            if len(ok_Action) > 1:
                # multiple robots need to go to their own goal simultaneously
                if len(RUN_TILL_SUCCESS_SET) == 1:
                    print("\nRobots {} can reach goal room simultaneously\n".format(sub_task_robot_num_set))
                    currentNBAState = nextNBAState
                    for robot_num in list(sub_task_robot_num_set):
                        robot_pos_dict[robot_num]['LAST_SUCCESS_q'] = robot_pos_dict_tmp[robot_num]['LAST_SUCCESS_q']
                        robot_pos_dict[robot_num]['LAST_SUCCESS_Q'] = robot_pos_dict_tmp[robot_num]['LAST_SUCCESS_Q']
                else:
                    print("\nRobot {} failed to reach simultaneously ===> Steps: {}\n".format(sub_task_robot_num_set,
                                                                                              RUN_TILL_SUCCESS_SET))
                    flagAlternate = buchi.update_alternate_transition(currentNBAState, nextNBAState)
                    if not flagAlternate:
                        is_verification_failed = buchi.delete_transition(currentNBAState, nextNBAState)
                        if is_verification_failed:
                            print("\n[Buchi] No more transitions to accepting state, verification failed.")
                            exit(1)
            else:
                # only single command needs to be satisfied.
                currentNBAState = nextNBAState
                for robot_num in list(sub_task_robot_num_set):
                    robot_pos_dict[robot_num]['LAST_SUCCESS_q'] = robot_pos_dict_tmp[robot_num]['LAST_SUCCESS_q']
                    robot_pos_dict[robot_num]['LAST_SUCCESS_Q'] = robot_pos_dict_tmp[robot_num]['LAST_SUCCESS_Q']


if __name__ == '__main__':
    main()
