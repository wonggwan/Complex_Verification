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
HORIZON = 12
BATCH_SIZE = HORIZON
SAMPLE_RATE = 0.1  # ts
net_dims = [INPUT_SIZE, 40, 40, OUTPUT_SIZE]
data_folder_name = './output/ground/con0_ts0.1_n12'
train_loader, test_loader = create_ground_data_loaders(BATCH_SIZE, data_folder_name, is_eval=True)
criterion = nn.MSELoss()
model_location = './output/ground/con0_ts0.1_n12/ground_mpc_py.pth'
model = Network(net_dims, activation=nn.ReLU).net
model.load_state_dict(torch.load(model_location))
model.eval()
u_range = torch.tensor([[-1, -1], [1, 1]])
goal_min = np.array([-0.5, -0.5, 0])  # min value of goal room
goal_max = np.array([0.5, 0.5, 2*np.pi])  # max value of goal room

# goal_range = np.array([goal_min, goal_max])
# num = 1
# for i in range(num):
#     x = np.random.uniform(low=goal_range[0, :], high=goal_range[1, :], size=(3))
#     x = torch.Tensor(x)
#     print(x)
#     for j in range(HORIZON):
#         pred = model(x)
#         x_next = ground_eval(x, pred, SAMPLE_RATE)
#         x = x_next
#         print(x)

check_arrival = 0
acc_list = []
print("length of test_loader: {}".format(len(test_loader)))
for k, (data, target) in enumerate(test_loader):
    state_list, pred_list, cnt, acc_cal = [], [], 0, 0
    cur_x = data[0]
    state_list.append(cur_x)
    for j in range(1, HORIZON):
        pred = model(cur_x)
        pred_list.append(pred)
        x_next = ground_eval(cur_x, pred, SAMPLE_RATE)
        state_list.append(x_next)
        cur_x = x_next
        acc_cal += 1 - criterion(x_next, data[j])
        cnt += 1

        # if j == HORIZON-1:
        #     print(x_next)

        if compare(x_next, goal_min, goal_max):
            check_arrival += 1
            break
    acc_list.append(acc_cal / cnt)
    # if k == 10:
    #     break
print('Trace accuracy: {}'.format(sum(acc_list) / len(acc_list)))
print('Number of times in goal region: {}'.format(check_arrival))

