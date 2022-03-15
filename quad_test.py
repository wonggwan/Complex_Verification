import torch
import torch.nn as nn
import numpy as np
from util.train_helper import Network, system, create_mpc_data_loaders

HORIZON = 20
SAMPLE_RATE = 0.3  # ts
net_dims = [6, 30, 30, 3]
BATCH_SIZE = HORIZON
"""
2.2091
    2.2488
    2.2195
   -1.0248
   -0.9896
   -1.0325
"""

"""
write a accuracy checker for the trained NN controller on the dataset being used
see which of these initial  state can end up being in the terminal constraint
"""


def main():
    model = Network(net_dims, activation=nn.ReLU).net
    model = model.cuda()
    checkpoint = torch.load('./output/quad_mpc_py.pth')
    model.load_state_dict(checkpoint)

    train_loader, test_loader = create_mpc_data_loaders(BATCH_SIZE)

    test_val = np.array([2, 0, 4, -0.5, -0.5, -0.5])
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
