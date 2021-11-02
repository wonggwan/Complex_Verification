import time
import matlab.engine
import numpy as np
import multi_MBA.test_code as te


def main():
    # Run multi_MBA code to get detailed information for each sub-task
    te.run_test()



    eng = matlab.engine.start_matlab()
    # A = matlab.double([[-0.5, 0], [0.1, -0.2]])
    # B = matlab.double([[0.5], [0.9]])
    # eng.main(nargout=0)
    eng.quit()


if __name__ == '__main__':
    main()
