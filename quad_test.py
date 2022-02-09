import torch
import torch.nn as nn
import numpy as np
from util.train_helper import Network, system

HORIZON = 20
SAMPLE_RATE = 0.3  # ts
net_dims = [6, 50, 50, 3]


def main():
    model = Network(net_dims, activation=nn.ReLU).net
    model = model.cuda()
    checkpoint = torch.load('./output/quad_mpc_py.pth')
    model.load_state_dict(checkpoint)

    test_val = np.array([-3, -2, -2, -0.7803, 0.4403, 0.5851])
    x = torch.FloatTensor(test_val).cuda()
    print(x)
    x = x.unsqueeze(0)
    for _ in range(HORIZON):
        x, u = x.squeeze(0), model(x).squeeze(0)
        res_gt = system(x.unsqueeze(1), u.unsqueeze(1), SAMPLE_RATE)
        print(res_gt)
        x = res_gt.unsqueeze(0).squeeze(2)


if __name__ == '__main__':
    main()
