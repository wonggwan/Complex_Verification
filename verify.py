import time
import matlab.engine
import multi_MBA.test_code as te


def main():
    # Run multi_MBA code to get detailed information for each sub-task
    # te.run_test()

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
        """
        Sub-task 2
        (successful goal)
            x: [2.45, 2.55, 2.55, 2.45]
            y: [2.1, 2.1, 2.3, 2.3]
        
        r2: [2.5; 2.2] (room2.mat / room2_y.mat / room2_x.mat)
        """
        print("\nSub-task 2: go to room 2 and avoid room 5")
        controller_1 = './output/room2.mat'
        room2_goal_x = matlab.double([2.45, 2.55, 2.55, 2.45])
        room2_goal_y = matlab.double([2.1, 2.1, 2.3, 2.3])
        eng = matlab.engine.start_matlab()
        result = eng.rsdp(A, B, X_poly_2, avoid_x, avoid_y, room2_goal_x, room2_goal_y, controller_1, nargout=2)
        eng.quit()
        is_satisfied_2 = result[0]
        FRS_V_bd_2 = result[1]
        if not is_satisfied_2:
            print("\nSub-task 2 failed, querying another sub-task...\n")
        else:
            print("\nSub-task 2 satisfied, querying next sub-task\n")


if __name__ == '__main__':
    main()
