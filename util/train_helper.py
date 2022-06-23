import os
import torch
import torch.nn as nn
from scipy.io import savemat
import numpy as np
import scipy.io
from torch.utils.data import Dataset
import matlab.engine


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


def create_ground_data_loaders(BATCH_SIZE, data_folder_name, is_eval=False):
    data_x_location = os.path.join(data_folder_name, 'ground_mpc_x.mat')
    data_y_location = os.path.join(data_folder_name, 'ground_mpc_y.mat')

    xmat = scipy.io.loadmat(data_x_location)['X_train_nnmpc']
    ymat = scipy.io.loadmat(data_y_location)['y_train_nnmpc']

    data_size = len(xmat)

    class GroundDataset(Dataset):
        def __init__(self, xmat, ymat):
            self.xmat = xmat
            self.ymat = ymat

        def __len__(self):
            return len(self.xmat)

        def __getitem__(self, index):
            image = torch.FloatTensor(self.xmat[index])
            label = torch.Tensor(np.array(self.ymat[index]))

            return image, label

    train_set = GroundDataset(xmat[:int(0.8*data_size)], ymat[:int(0.8*data_size)])
    test_set = GroundDataset(xmat[int(0.8*data_size):], ymat[int(0.8*data_size):])
    train_loader = torch.utils.data.DataLoader(train_set, batch_size=BATCH_SIZE, shuffle=False, num_workers=0,
                                               drop_last=True)
    if is_eval:
        test_loader = torch.utils.data.DataLoader(test_set, batch_size=BATCH_SIZE, shuffle=False, num_workers=0,
                                                  drop_last=True)
    else:
        test_loader = torch.utils.data.DataLoader(test_set, batch_size=1, shuffle=False, num_workers=0,
                                                  drop_last=True)
    print(len(train_set), len(test_set))
    return train_loader, test_loader


def create_mpc_data_loaders(BATCH_SIZE, data_folder_name, is_eval=False):
    data_x_location = os.path.join(data_folder_name, 'quad_mpc_x.mat')
    data_y_location = os.path.join(data_folder_name, 'quad_mpc_y.mat')
    xmat = scipy.io.loadmat(data_x_location)['X_train_nnmpc']
    ymat = scipy.io.loadmat(data_y_location)['y_train_nnmpc']
    data_size = len(xmat)

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

    train_set = MyDataset(xmat[:int(0.05 * data_size)], ymat[:int(0.05 * data_size)])
    test_set = MyDataset(xmat[int(0.8 * data_size):], ymat[int(0.8 * data_size):])
    train_loader = torch.utils.data.DataLoader(train_set, batch_size=BATCH_SIZE, shuffle=False, num_workers=0,
                                               drop_last=True)
    if is_eval:
        test_loader = torch.utils.data.DataLoader(test_set, batch_size=BATCH_SIZE, shuffle=False, num_workers=0, drop_last=False)
    else:
        test_loader = torch.utils.data.DataLoader(test_set, batch_size=1, shuffle=False, num_workers=0,
                                                  drop_last=False)
    print(len(train_set))
    return train_loader, test_loader


def train_6d_ver1(BATCH_SIZE, INPUT_SIZE, OUTPUT_SIZE, NUM_EPOCHS, optimizer):
    train_loader, test_loader = create_mpc_data_loaders(BATCH_SIZE)

    net_dims = [INPUT_SIZE, 35, 35, OUTPUT_SIZE]
    model = Network(net_dims, activation=nn.ReLU).net
    model = model.cuda()

    criterion = nn.L1Loss(reduction='mean')
    # criterion = nn.MSELoss()
    # optimizer = optim.Adam(model.parameters(), lr=LEARNING_RATE)

    for epoch_num in range(1, NUM_EPOCHS + 1):
        mean_loss, cnt = 0, 0
        print("Epoch -> {}".format(epoch_num))
        model.train()
        for batch_idx, (data, target) in enumerate(train_loader):
            cnt += 1
            data, target = data.cuda(), target.cuda()
            data = data.view(BATCH_SIZE, -1)
            optimizer.zero_grad()
            output = model(data)
            loss = criterion(output, target)
            loss.backward()
            optimizer.step()
            mean_loss += loss
        print("Mean Loss -> {}\n\n".format(mean_loss / cnt))

    weights, biases = extract_weights(model)
    data = {'weights': np.array(weights, dtype=object), 'biases': np.array(biases, dtype=object)}

    fname = './output/quad_mpc.mat'
    savemat(fname, data)
    # test_model_6d(model, test_loader)


def test_model_6d(model, test_loader, horizon, ts):
    model.eval()
    model.cpu()
    count = 0
    eng = matlab.engine.start_matlab()
    with torch.no_grad():

        for data, labels in test_loader:
            print("Count -> {}".format(count))

            x0 = data.detach().numpy().transpose()
            print("x0: ", x0)

            for _ in range(horizon):
                data = torch.from_numpy(x0)
                data = torch.transpose(data, 0, 1)
                data = data.to(torch.float32)
                output = model(data)
                u = output.detach().numpy().transpose()

                x0_matlab = matlab.double(x0.tolist())
                u_matlab = matlab.double(u.tolist())
                result = eng.test_6d_controller(x0_matlab, u_matlab, ts, nargout=1)
                x = np.asarray(result)

                print("\nx: ", x)
                x0 = x

            break

    eng.quit()


def ground_eval(x, u, ts):
    res = torch.zeros_like(x)
    tmp = u[1] * ts / 2 + 1e-6
    sinc = torch.sin(tmp * np.pi) / (tmp * np.pi)
    cal = torch.multiply(u[0], sinc)
    res[0] = x[0] + torch.multiply(cal, torch.cos(x[2] + tmp))
    res[1] = x[1] + torch.multiply(cal, torch.sin(x[2] + tmp))
    res[2] = x[2] + ts * u[1]
    return res


def ground_dym_test(xv, uv, ts):
    res = torch.zeros_like(xv)
    # for j, (x, u) in enumerate(zip(xv, uv)):
    #     tmp = u[1] * ts / 2 + 1e-6
    #     sinc = torch.sin(tmp * np.pi) / (tmp * np.pi)
    #     cal = torch.multiply(u[0], sinc)
    #     res[j, 0] = x[0] + torch.multiply(cal, torch.cos(x[2] + tmp))
    #     res[j, 1] = x[1] + torch.multiply(cal, torch.sin(x[2] + tmp))
    #     res[j, 2] = x[2] + ts * u[1]
    tmp = uv[:, 1] * ts / 2 + 1e-6
    sinc = torch.div(torch.sin(tmp * np.pi), (tmp * np.pi))
    cal = torch.multiply(uv[:, 0], sinc)
    res[:, 0] = xv[:, 0] + torch.multiply(cal, torch.cos(xv[:, 2] + tmp))
    res[:, 1] = xv[:, 1] + torch.multiply(cal, torch.sin(xv[:, 2] + tmp))
    res[:, 2] = xv[:, 2] + ts * uv[:, 1]
    return res


def ground_dym(xv, uv, ts):
    res = torch.zeros_like(xv).cuda()
    # for j, (x, u) in enumerate(zip(xv, uv)):
    #     tmp = u[1] * ts / 2 + 1e-6
    #     sinc = torch.sin(tmp * np.pi) / (tmp * np.pi)
    #     cal = torch.multiply(u[0], sinc)
    #     sinc, cal = sinc.cuda(), cal.cuda()
    #     res[j, 0] = x[0] + torch.multiply(cal, torch.cos(x[2] + tmp))
    #     res[j, 1] = x[1] + torch.multiply(cal, torch.sin(x[2] + tmp))
    #     res[j, 2] = x[2] + ts * u[1]
    tmp = uv[:, 1] * ts / 2 + 1e-6
    sinc = torch.div(torch.sin(tmp * np.pi), (tmp * np.pi))
    cal = torch.multiply(uv[:, 0], sinc)
    res[:, 0] = xv[:, 0] + torch.multiply(cal, torch.cos(xv[:, 2] + tmp))
    res[:, 1] = xv[:, 1] + torch.multiply(cal, torch.sin(xv[:, 2] + tmp))
    res[:, 2] = xv[:, 2] + ts * uv[:, 1]
    return res


def dym_for_train(xv, uv, ts):
    res = torch.zeros_like(xv).cuda()
    tmp = uv[1] * ts / 2 + 1e-6
    sinc = torch.sin(tmp * np.pi) / (tmp * np.pi)
    cal = uv[0] * sinc
    res[0] = xv[0] + cal * torch.cos(xv[2] + tmp)
    res[1] = xv[1] + cal * torch.sin(xv[2] + tmp)
    res[2] = xv[2] + ts * uv[1]
    return res


def system(x, u, ts):
    global A, B, E
    # ts=0.5
    g = 9.81
    if ts == 0.5:
        # print("ts={}".format(ts))
        A = torch.tensor([[1, 0, 0, 0.5, 0, 0],
                          [0, 1, 0, 0, 0.5, 0],
                          [0, 0, 1, 0, 0, 0.5],
                          [0, 0, 0, 1, 0, 0],
                          [0, 0, 0, 0, 1, 0],
                          [0, 0, 0, 0, 0, 1]]).cuda()
        B = torch.tensor([[1.2263, 0, 0],
                          [0, -1.2263, 0],
                          [0, 0, 0.125],
                          [4.9050, 0, 0],
                          [0, -4.9050, 0],
                          [0, 0, 0.5]]).cuda()
        E = torch.tensor([[0], [0], [-0.125], [0], [0], [-0.5]]).cuda()
    elif ts == 0.2:
        A = torch.tensor([[1, 0, 0, 0.2, 0, 0],
                          [0, 1, 0, 0, 0.2, 0],
                          [0, 0, 1, 0, 0, 0.2],
                          [0, 0, 0, 1, 0, 0],
                          [0, 0, 0, 0, 1, 0],
                          [0, 0, 0, 0, 0, 1]]).cuda()
        B = torch.tensor([[0.1962, 0, 0],
                          [0, -0.1962, 0],
                          [0, 0, 0.02],
                          [1.962, 0, 0],
                          [0, -1.962, 0],
                          [0, 0, 0.2]]).cuda()
        E = torch.tensor([[0], [0], [-0.02], [0], [0], [-0.2]]).cuda()
    elif ts == 0.3:
        # print("ts={}".format(ts))
        A = torch.tensor([[1, 0, 0, 0.3, 0, 0],
                          [0, 1, 0, 0, 0.3, 0],
                          [0, 0, 1, 0, 0, 0.3],
                          [0, 0, 0, 1, 0, 0],
                          [0, 0, 0, 0, 1, 0],
                          [0, 0, 0, 0, 0, 1]]).cuda()
        B = torch.tensor([[0.4415, 0, 0],
                          [0, -0.4415, 0],
                          [0, 0, 0.045],
                          [2.943, 0, 0],
                          [0, -2.943, 0],
                          [0, 0, 0.3]]).cuda()
        E = torch.tensor([[0], [0], [-0.045], [0], [0], [-0.3]]).cuda()
    elif ts == 0.1:
        # print("ts={}".format(ts))
        A = torch.tensor([[1, 0, 0, 0.1, 0, 0],
                          [0, 1, 0, 0, 0.1, 0],
                          [0, 0, 1, 0, 0, 0.1],
                          [0, 0, 0, 1, 0, 0],
                          [0, 0, 0, 0, 1, 0],
                          [0, 0, 0, 0, 0, 1]]).cuda()
        B = torch.tensor([[0.0491, 0, 0],
                          [0, -0.0491, 0],
                          [0, 0, 0.005],
                          [0.981, 0, 0],
                          [0, -0.981, 0],
                          [0, 0, 0.1]]).cuda()
        E = torch.tensor([[0], [0], [-0.005], [0], [0], [-0.1]]).cuda()
    res = torch.matmul(A, x) + torch.matmul(B, u) + E * g
    return res
