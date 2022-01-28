import torch
import torch.nn as nn
import torch.optim as optim
from scipy.io import savemat
import numpy as np
# import control
import matlab.engine
from util.train_helper import Network, extract_weights, create_mpc_data_loaders, test_model_6d

INPUT_SIZE = 6
OUTPUT_SIZE = 3
BATCH_SIZE = 100
NUM_EPOCHS = 50
LEARNING_RATE = 0.001
HORIZON = 10


def system(x, u):
    g = 9.81
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
    res = torch.matmul(A, x) + torch.matmul(B, u) + E * g
    return res


def main():
    train_loader, test_loader = create_mpc_data_loaders(BATCH_SIZE)
    # net_dims = [INPUT_SIZE, 100, 50, 50, 20, OUTPUT_SIZE]
    net_dims = [INPUT_SIZE, 35, 35, OUTPUT_SIZE]
    model = Network(net_dims, activation=nn.ReLU).net
    model = model.cuda()
    # criterion = nn.L1Loss(reduction='mean')
    criterion = nn.MSELoss()
    optimizer = optim.Adam(model.parameters(), lr=LEARNING_RATE)
    for epoch_num in range(1, NUM_EPOCHS + 1):
        mean_loss, cnt = 0, 0
        print("Epoch -> {}".format(epoch_num))
        model.train()
        for batch_idx, (data, target) in enumerate(train_loader):
            data, target = data.cuda(), target.cuda()
            u_pred = model(data)
            u_pred = u_pred.cuda()
            sloss = 0
            for x, ugt, u in zip(data, target, u_pred):
                res_gt = system(x.unsqueeze(1), ugt.unsqueeze(1))
                res_pred = system(x.unsqueeze(1), u.unsqueeze(1))
                sloss += criterion(res_gt, res_pred)
            loss = sloss / data.size(0)
            mean_loss += loss
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            cnt += 1
        val = mean_loss / cnt
        val = float(val.cpu().detach().numpy())
        print("mean loss->", val, "\n")

    weights, biases = extract_weights(model)
    data = {'weights': np.array(weights, dtype=object), 'biases': np.array(biases, dtype=object)}

    fname = './output/quad_mpc.mat'
    savemat(fname, data)

    test_model_6d(model, test_loader, HORIZON)


if __name__ == '__main__':
    main()
