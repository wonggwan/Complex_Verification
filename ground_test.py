import numpy as np
import torch
import torch.nn as nn
from util.train_helper import Network, create_ground_data_loaders, ground_eval


def compare(x, min_region, max_region):
    if min_region[0] <= x[0] <= max_region[0] and \
            min_region[1] <= x[1] <= max_region[1]:
        return True
    return False


INPUT_SIZE = 3
OUTPUT_SIZE = 2
HORIZON = 50
BATCH_SIZE = HORIZON
SAMPLE_RATE = 1  # ts
net_dims = [INPUT_SIZE, 40, 40, OUTPUT_SIZE]
data_folder_name = './output/ground/data1_ts_1_n15'
# train_loader, test_loader = create_ground_data_loaders(BATCH_SIZE, data_folder_name, is_eval=True)
criterion = nn.MSELoss()
model_location = './output/ground/data1_ts_1_n15/ground_mpc_py.pth'
model = Network(net_dims, activation=nn.ReLU).net
model.load_state_dict(torch.load(model_location))
model.eval()
u_range = torch.tensor([[-0.22, -0.15], [0.22, 0.15]])

goal_min = np.array([0.9, 0.9])  # min value of goal room
goal_max = np.array([1.1, 1.1])  # max value of goal room

work_space_min = np.array([2.98, 2.98, 3.92])
work_space_max = np.array([3.02, 3.02, 3.94])

# goal_min = np.array([-0.2, -0.2, 1])  # min value of goal room
# goal_max = np.array([0.2, 0.2, 4])  # max value of goal room
#
# work_space_min = np.array([0.93, 0.95, 2.20])
# work_space_max = np.array([1.05, 1.16, 2.40])

work_range = np.array([work_space_min, work_space_max])

tss = 10
check_arrival = 0
state_list, pred_list = [], []
num = 0
while num < 1:
    state_list_, pred_list_ = [], []
    cnt, acc_cal = 0, 0
    x0 = np.random.uniform(low=work_range[0, :], high=work_range[1, :], size=3)
    cur_x = torch.Tensor(x0)
    state_list_.append(cur_x)
    is_check = False
    for j in range(1, HORIZON):
        if is_check:
            break
        pred = model(cur_x)
        pred = torch.clamp(pred, min=u_range[0, :], max=u_range[1, :])
        x_next = ground_eval(cur_x, pred, SAMPLE_RATE)
        pred_list_.append(list(np.array(pred.detach())))
        state_list_.append(x_next)
        cur_x = x_next
        if compare(x_next, goal_min, goal_max):
            check_arrival += 1
            state_list.append(state_list_)
            pred_list.append(pred_list_)
            is_check = True
            print("Current success: {}\n".format(x0))
            print("State List: ")
            for element in state_list_:
                print(element)
            print("Time: {}".format(len(pred_list_)))
            print("\nPrediction: ")
            for element in pred_list_:
                print(element)
            print("\n\n")
            print(pred_list_)
            num += 1
            break

# print('Number of times in goal region: {}'.format(check_arrival))
# for state in state_list:
#     cnt = 0
#     for s in state:
#         if cnt % 10 == 0 and cnt != 0:
#             print("----------------------------")
#         cnt += 1
#         print(s)
#     print("\n")
