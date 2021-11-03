import time
import matlab.engine
import multi_MBA.test_code as mba


def main():
    # Run multi_MBA code to get detailed information for each sub-task
    # mba.run_test()

    print("Overall Command:\n1) Go to room 1 and avoid room 5\n2) Go to room 2 or room 3 and avoid room 5\n3) Go to "
          "room 4 and avoid room 5")
    """
    System dynamics
    
    Avoiding room 5: 
        x: [1, 1.2, 1.2, 1]
        y: [1, 1, 1.2, 1.2]
    
    Initial set of states:
        x: [1, 1.5, 1.5, 1]
        y: [2, 2, 2.5, 2.5]
    """
    A = matlab.double([[-0.5, 0], [0.1, -0.2]])
    B = matlab.double([[1, 0], [0, 1]])

    avoid_x = matlab.double([1, 1.2, 1.2, 1])
    avoid_y = matlab.double([1, 1, 1.2, 1.2])

    X_poly = matlab.double([[1, 2], [1.5, 2], [1.5, 2.5], [1, 2.5]])

    """
    Sub-task 1
    (successful goal)
        x: [1.95, 2.05, 2.05, 1.95]
        y: [0.98, 0.98, 1.02, 1.02]
    
    r1: [2; 1]
    """
    print("\nSub-task 1: go to room 1 and avoid room 5")
    controller_1 = './output/room1.mat'
    room1_goal_x = matlab.double([1.95, 2.05, 2.05, 1.95])
    room1_goal_y = matlab.double([0.98, 0.98, 1.02, 1.02])
    eng = matlab.engine.start_matlab()
    result = eng.rsdp(A, B, X_poly, avoid_x, avoid_y, room1_goal_x, room1_goal_y, controller_1, nargout=2)
    eng.quit()
    is_satisfied_1 = result[0]
    FRS_V_bd_1 = result[1]
    if not is_satisfied_1:
        print("\nSub-task 1 not satisfied, verification failed\n")
    else:
        X_poly_2 = FRS_V_bd_1
        print("\nSub-task 1 satisfied, querying next sub-task\n")
        """
        Sub-task 2
        (successful goal)
            x: [2.45, 2.55, 2.55, 2.45]
            y: [2.1, 2.1, 2.3, 2.3]
        (fail goal)
            x: [3.45, 3.55, 3.55, 3.45]
            y: [3.1, 3.1, 3.3, 3.3]
        
        r2: [2.5; 2.2] 
        """
        print("\nSub-task 2: go to room 2 and avoid room 5")
        controller_2 = './output/room2.mat'
        # successful case
        # room2_goal_x = matlab.double([2.45, 2.55, 2.55, 2.45])
        # room2_goal_y = matlab.double([2.1, 2.1, 2.3, 2.3])
        # fail case
        room2_goal_x = matlab.double([3.45, 3.55, 3.55, 3.45])
        room2_goal_y = matlab.double([3.1, 3.1, 3.3, 3.3])
        eng = matlab.engine.start_matlab()
        result = eng.rsdp(A, B, X_poly_2, avoid_x, avoid_y, room2_goal_x, room2_goal_y, controller_2, nargout=2)
        eng.quit()
        is_satisfied_2 = result[0]
        FRS_V_bd_2 = result[1]
        if is_satisfied_2:
            X_poly_3 = FRS_V_bd_2
            print("\nSub-task 2 satisfied, querying next sub-task\n")
            """
            Sub-task 3
            (successful goal)
                x: [-1.01, -0.98, -0.98, -1.01]
                y: [-1.01, -1.01, -0.98, -0.98]

            r4: [-1; -1]
            """
            print("\nSub-task 3: go to room 4 and avoid room 5")
            controller_4 = './output/room4.mat'
            room4_goal_x = matlab.double([-1.01, -0.98, -0.98, -1.01])
            room4_goal_y = matlab.double([-1.01, -1.01, -0.98, -0.98])
            eng = matlab.engine.start_matlab()
            result = eng.rsdp(A, B, X_poly_3, avoid_x, avoid_y, room4_goal_x, room4_goal_y, controller_4, nargout=2)
            eng.quit()
            is_satisfied_3 = result[0]
            if is_satisfied_3:
                print("\nSub-task 3 satisfied, verification succeeded.\n")
            else:
                print("\nSub-task 3 not satisfied, verification failed\n")
        else:
            print("\nSub-task 2 failed, querying another sub-task...\n")
            X_poly_4 = FRS_V_bd_2
            """
            Sub-task 4
            (successful goal)
                x: [1.18, 1.25, 1.25, 1.18]
                y: [1.38, 1.38, 1.42, 1.42]
                
            r3: [1.2; 1.4]
            """
            print("\nSub-task 4: go to room 3 and avoid room 5")
            controller_3 = './output/room3.mat'
            room3_goal_x = matlab.double([1.18, 1.25, 1.25, 1.18])
            room3_goal_y = matlab.double([1.38, 1.38, 1.42, 1.42])
            eng = matlab.engine.start_matlab()
            result = eng.rsdp(A, B, X_poly_4, avoid_x, avoid_y, room3_goal_x, room3_goal_y, controller_3, nargout=2)
            eng.quit()
            is_satisfied_4 = result[0]
            FRS_V_bd_4 = result[1]
            if not is_satisfied_4:
                print("\nSub-task 4 not satisfied, verification failed\n")
            else:
                X_poly_5 = FRS_V_bd_4
                """
                Sub-task 5
                (successful goal)
                    x: [-1.01, -0.98, -0.98, -1.01]
                    y: [-1.01, -1.01, -0.98, -0.98]
    
                r4: [-1; -1]
                """
                print("\nSub-task 5: go to room 4 and avoid room 5")
                controller_4 = './output/room4.mat'
                room4_goal_x = matlab.double([-1.01, -0.98, -0.98, -1.01])
                room4_goal_y = matlab.double([-1.01, -1.01, -0.98, -0.98])
                eng = matlab.engine.start_matlab()
                result = eng.rsdp(A, B, X_poly_5, avoid_x, avoid_y, room4_goal_x, room4_goal_y, controller_4, nargout=2)
                eng.quit()
                is_satisfied_5 = result[0]
                if is_satisfied_5:
                    print("\nSub-task 5 satisfied, verification succeeded.")
                else:
                    print("\nSub-task 5 not satisfied, verification failed\n")


if __name__ == '__main__':
    main()
