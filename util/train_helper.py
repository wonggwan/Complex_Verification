import torch
import torch.nn as nn
from scipy.io import savemat
import numpy as np
import scipy.io
from torch.utils.data import Dataset


class Network(nn.Module):
    def __init__(self, net_dims, activation=nn.ReLU):
        super(Network, self).__init__()
        layers = []
        for i in range(len(net_dims) - 1):
            layers.append(nn.Linear(net_dims[i], net_dims[i + 1]))
            if i != len(net_dims) - 2:
                layers.append(activation())
        self.net = nn.Sequential(*layers)

    def forward(self, x):
        return self.net(x)


def extract_weights(net):
    weights = []
    biases = []
    for param_tensor in net.state_dict():
        tensor = net.state_dict()[param_tensor].detach().cpu().numpy().astype(np.float64)
        if 'weight' in param_tensor:
            weights.append(tensor)
        if 'bias' in param_tensor:
            tensor = tensor.transpose(0)
            tensor = tensor.reshape(-1, 1)
            biases.append(tensor)
    return weights, biases


def create_mpc_data_loaders(BATCH_SIZE):
    xmat = scipy.io.loadmat('../output/quad_mpc_x.mat')['X_train_nnmpc']
    ymat = scipy.io.loadmat('../output/quad_mpc_y.mat')['y_train_nnmpc']

    class MyDataset(Dataset):
        def __init__(self, xmat, ymat):
            self.xmat = xmat
            self.ymat = ymat

        def __len__(self):
            return len(self.xmat)

        def __getitem__(self, index):
            image = torch.FloatTensor(self.xmat[index])
            label = torch.Tensor(np.array(self.ymat[index]))
            return image, label

    train_set = MyDataset(xmat, ymat)
    test_set = MyDataset(xmat, ymat)
    train_loader = torch.utils.data.DataLoader(train_set, batch_size=BATCH_SIZE, shuffle=False, num_workers=0,
                                               drop_last=True)
    test_loader = torch.utils.data.DataLoader(test_set, batch_size=1, shuffle=False, num_workers=0, drop_last=False)
    print(len(train_set))
    return train_loader, test_loader