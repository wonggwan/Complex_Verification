import torch
import torch.nn as nn
import numpy as np
from util.train_helper import Network, system, create_mpc_data_loaders


def compare(x, min_region, max_region):
    if min_region[0] <= x[0] <= max_region[0] and \
            min_region[1] <= x[1] <= max_region[1] and \
            min_region[2] <= x[2] <= max_region[2]:
        return True
    return False


HORIZON = 20
SAMPLE_RATE = 0.3  # ts
net_dims = [6, 30, 30, 3]
BATCH_SIZE = HORIZON
data_folder_name = './output/con1'
train_loader, test_loader = create_mpc_data_loaders(BATCH_SIZE, data_folder_name, is_eval=True)
criterion = nn.MSELoss()
model_location = './output/con1/quad_mpc_py.pth'
model = Network(net_dims, activation=nn.ReLU).net
model.load_state_dict(torch.load(model_location))
model = model.cuda()
model.eval()

goal_min = np.array([0.5, 0.5, 0.5, -1, -1, -1])  # max value of goal room
goal_max = np.array([1.5, 1.5, 1.5, 1, 1, 1])  # min value of goal room

goal_range = np.array([goal_min, goal_max])


# num = 1
# for i in range(num):
#     x = np.random.uniform(low=goal_range[0, :], high=goal_range[1, :], size=(1, 6))
#     x = torch.Tensor(x).cuda()
#     for j in range(HORIZON):
#         print(x)
#         pred = model(x)
#         pred = pred.squeeze(0).unsqueeze(1)
#         x = x.squeeze(0).unsqueeze(1)
#         x_next = system(x, pred, SAMPLE_RATE)
#         x = x_next.squeeze(1).unsqueeze(0)

check_arrival = 0
acc_list = []
print("length of test_loader: {}".format(len(test_loader)))
for k, (data, target) in enumerate(test_loader):
    state_list, pred_list, cnt, acc_cal = [], [], 0, 0
    data, target = data.cuda(), target.cuda()
    cur_x = data[0].unsqueeze(0)
    state_list.append(cur_x)
    for j in range(1, HORIZON):
        pred = model(cur_x)
        pred_list.append(pred)
        pred = pred.squeeze(0).unsqueeze(1)
        cur_x = cur_x.squeeze(0).unsqueeze(1)
        x_next = system(cur_x, pred, SAMPLE_RATE)
        state_list.append(x_next)
        cur_x = x_next.squeeze(1).unsqueeze(0)
        acc_cal += 1 - criterion(x_next, data[j].unsqueeze(1))
        cnt += 1
        # if j == HORIZON-1:
        #     print(x_next)
        if compare(x_next, goal_min, goal_max):
            check_arrival += 1
            break
    acc_list.append(acc_cal / cnt)

print('Trace accuracy: {}'.format(sum(acc_list) / len(acc_list)))
print('Number of times in goal region: {}'.format(check_arrival))
