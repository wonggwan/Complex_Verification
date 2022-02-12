import torch
import torch.nn as nn
import numpy as np
from util.train_helper import Network, system

HORIZON = 20
SAMPLE_RATE = 0.3  # ts
net_dims = [6, 100, 50, 3]

"""
2.2091
    2.2488
    2.2195
   -1.0248
   -0.9896
   -1.0325
"""

def main():
    model = Network(net_dims, activation=nn.ReLU).net
    model = model.cuda()
    checkpoint = torch.load('./output/quad_mpc_py.pth')
    model.load_state_dict(checkpoint)

    test_val = np.array([4.5,4.5,0.8,0,0,0])
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
