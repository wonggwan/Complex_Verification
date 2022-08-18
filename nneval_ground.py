import torch
import torch.nn as nn
import numpy as np
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
train_loader, test_loader = create_ground_data_loaders(BATCH_SIZE, data_folder_name, is_eval=True)
criterion = nn.MSELoss()
model_location = './output/ground/data1_ts_1_n15/ground_mpc_py.pth'
model = Network(net_dims, activation=nn.ReLU).net
model.load_state_dict(torch.load(model_location))
model.eval()

# goal_min = np.array([0.8, 0.8])  # min value of goal room
# goal_max = np.array([1.2, 1.2])  # max value of goal room

goal_min = np.array([-0.1, -0.1])  # min value of goal room
goal_max = np.array([0.1, 0.1])  # max value of goal room

u_range = torch.tensor([[-0.22, -0.15], [0.22, 0.15]])
goal_range = np.array([goal_min, goal_max])

check_arrival = 0
print("length of test_loader: {}".format(len(test_loader)))
state_list, pred_list = [], []
for k, (data, target) in enumerate(test_loader):
    state_list_, pred_list_, cnt, acc_cal = [], [], 0, 0
    cur_x = data[0]
    state_list_.append(cur_x)
    for j in range(1, HORIZON):
        pred = model(cur_x)
        pred = torch.clamp(pred, min=u_range[0, :], max=u_range[1, :])
        x_next = ground_eval(cur_x, pred, SAMPLE_RATE)
        pred_list_.append(list(np.array(pred.detach())))
        state_list_.append(x_next)
        cur_x = x_next
        cnt += 1
        if compare(x_next, goal_min, goal_max):
            check_arrival += 1
            break

print('Number of times in goal region: {}'.format(check_arrival))
