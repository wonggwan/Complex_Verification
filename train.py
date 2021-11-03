import torch
import torch.nn as nn
import torch.optim as optim
from scipy.io import savemat
import numpy as np
import scipy.io
from torch.utils.data import Dataset

INPUT_SIZE = 2
OUTPUT_SIZE = 2
BATCH_SIZE = 100
NUM_EPOCHS = 100
LEARNING_RATE = 0.001


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


def train_network(model, train_loader):
    criterion = nn.L1Loss(reduction='mean')
    optimizer = optim.Adam(model.parameters(), lr=LEARNING_RATE)

    mean_loss = 0
    cnt = 0
    for epoch_num in range(1, NUM_EPOCHS + 1):
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
    print("\n\n Mean Loss -> {}".format(mean_loss / cnt))


def create_data_loaders(room_name = 1):
    xmat = scipy.io.loadmat('output/room'+str(room_name)+'_x.mat')['X_train_ri']
    ymat = scipy.io.loadmat('output/room'+str(room_name)+'_y.mat')['y_train_ri']

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


def test_model(model, test_loader):
    model.eval()
    model.cpu()
    with torch.no_grad():
        for data, labels in test_loader:
            x0 = data.detach().numpy().transpose()
            A = np.array([[-0.5, 0], [0.1, -0.2]])
            B = np.array([[1, 0], [0, 1]])
            # K1 = np.array([[0.3, 0.3], [0.2, 0.2]])
            # xe = np.array([[2], [1]])
            # BK2r = -1*np.matmul(A-np.matmul(B, K1)-np.identity(2), xe)
            # K2 = np.array([[3, 3]])
            # K2 = -1 * (A - np.matmul(B, K1) - np.identity(2))
            # r = np.array([[2], [1]])
            print("x0: ", x0)

            for _ in range(10):
                data = torch.from_numpy(x0)
                data = torch.transpose(data, 0, 1)
                data = data.to(torch.float32)
                output = model(data)
                u = output.detach().numpy().transpose()
                # x = np.matmul(A, x0) - np.matmul(B, u) + BK2r
                x = np.matmul(A, x0) + np.matmul(B, u)
                print("\nx: ", x)
                x0 = x
            break


def main():
    room_name = 2

    train_loader, test_loader = create_data_loaders(room_name)
    net_dims = [INPUT_SIZE, 20, 10, 5, OUTPUT_SIZE]
    model = Network(net_dims, activation=nn.ReLU).net
    model = model.cuda()
    train_network(model, train_loader)
    test_model(model, test_loader)
    weights, biases = extract_weights(model)
    data = {'weights': np.array(weights, dtype=np.object), 'biases': np.array(biases, dtype=np.object)}

    fname = './output/room' + str(room_name) + '.mat'
    savemat(fname, data)


if __name__ == '__main__':
    main()
