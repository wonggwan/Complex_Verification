import torch
import torch.nn as nn
import torch.optim as optim
from scipy.io import savemat
import numpy as np
import matlab.engine
from tqdm import tqdm
from util.train_helper import Network, extract_weights, create_mpc_data_loaders, system

INPUT_SIZE = 6
OUTPUT_SIZE = 3
HORIZON = 20
BATCH_SIZE = HORIZON
NUM_EPOCHS = 10
LEARNING_RATE = 0.001
SAMPLE_RATE = 0.3  # ts
net_dims = [INPUT_SIZE, 100, 50, OUTPUT_SIZE]


def main():
    train_loader, test_loader = create_mpc_data_loaders(BATCH_SIZE)
    model = Network(net_dims, activation=nn.ReLU).net
    model = model.cuda()
    criterion = nn.MSELoss()
    optimizer = optim.Adam(model.parameters(), lr=LEARNING_RATE)
    for epoch_num in tqdm(range(1, NUM_EPOCHS + 1)):
        mean_loss, cnt = 0, 0
        print("Epoch -> {}".format(epoch_num))
        model.train()
        for batch_idx, (data, target) in enumerate(train_loader):
            data, target = data.cuda(), target.cuda()
            u_pred = model(data)
            u_pred = u_pred.cuda()
            sloss = 0
            for x, ugt, u in zip(data, target, u_pred):
                res_gt = system(x.unsqueeze(1), ugt.unsqueeze(1), SAMPLE_RATE)
                res_pred = system(x.unsqueeze(1), u.unsqueeze(1), SAMPLE_RATE)
                sloss += criterion(res_gt, res_pred)
            loss = sloss / data.size(0)
            mean_loss += loss
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            cnt += 1
        # break
        val = mean_loss / cnt
        val = float(val.cpu().detach().numpy())
        print("mean loss->", val, "\n")

    weights, biases = extract_weights(model)
    data = {'weights': np.array(weights, dtype=object), 'biases': np.array(biases, dtype=object)}
    fname = './output/quad_mpc.mat'
    savemat(fname, data)
    save_path = './output/quad_mpc_py.pth'
    torch.save(model.state_dict(), save_path)
    model.eval()
    for batch_idx, (data, target) in enumerate(test_loader):
        data, target = data.cuda(), target.cuda()
        print(data)
        for _ in range(HORIZON):
            u_pred = model(data)
            u_pred = u_pred.cuda()
            for x, ugt, u in zip(data, target, u_pred):
                res_pred = system(x.unsqueeze(1), u.unsqueeze(1), SAMPLE_RATE)
                print(res_pred)
                data = res_pred.squeeze(1).unsqueeze(0)
        break


if __name__ == '__main__':
    main()
