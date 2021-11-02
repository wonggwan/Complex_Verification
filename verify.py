import time
import matlab.engine
import numpy as np
import multi_MBA.test_code as te


def main():
    # Run multi_MBA code to get detailed information for each sub-task
    te.run_test()

    # System Dynamics
    A = matlab.double([[-0.5, 0], [0.1, -0.2]])
    B = matlab.double([[1, 0], [0, 1]])

    # Sub-Task 1
    print("\nSub-task 1: go to room 1 and avoid room 5")

    init_set = matlab.double([[1.5], [-1.0], [2.5], [-2.0]])
    avoid_x = matlab.double([0.5, 0.5, 1, 1])
    avoid_y = matlab.double([-0.4, -0.8, -0.8, -0.4])
    goal_x = matlab.double([0.95, 1.05, 1.05, 0.95])
    goal_y = matlab.double([0.98, 0.98, 1.02, 1.02])
    eng = matlab.engine.start_matlab()
    is_satisfied = eng.rsdp(A, B, init_set, avoid_x, avoid_y, goal_x, goal_y)
    print(is_satisfied)
    # eng.run(nargout=0)
    eng.quit()


if __name__ == '__main__':
    main()
