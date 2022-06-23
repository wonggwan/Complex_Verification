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
NUM_EPOCHS = 120
LEARNING_RATE = 0.01
SAMPLE_RATE = 0.3  # ts
net_dims = [INPUT_SIZE, 30, 30, OUTPUT_SIZE]
data_location = './output/con0'
g = 9.81
u_range = torch.tensor([[-np.pi/9, -np.pi/9, 0], [np.pi/9, np.pi/9, 2*g]]).cuda()


def main():
    train_loader, test_loader = create_mpc_data_loaders(BATCH_SIZE, data_location)
    model = Network(net_dims, activation=nn.ReLU).net
    model = model.cuda()
    criterion = nn.MSELoss()
    optimizer = optim.Adam(model.parameters(), lr=LEARNING_RATE)
    scheduler = torch.optim.lr_scheduler.StepLR(optimizer, step_size=30, gamma=0.3)
    for epoch_num in tqdm(range(1, NUM_EPOCHS + 1)):
        mean_loss, cnt = 0, 0
        print("Epoch -> {}".format(epoch_num))
        model.train()
        for batch_idx, (data, target) in enumerate(train_loader):
            data, target = data.cuda(), target.cuda()
            # u_pred = model(data)
            # u_pred = u_pred.cuda()
            # # loss = criterion(target, u_pred)

            # New Training Method
            xprev = data[:BATCH_SIZE - 1]
            pred = model(xprev)
            pred = torch.clamp(pred, min=u_range[0, :], max=u_range[1, :])
            xprev_t = torch.transpose(xprev, 0, 1)
            pred_t = torch.transpose(pred, 0, 1)
            xnext = torch.transpose(system(xprev_t, pred_t, SAMPLE_RATE), 0, 1)
            loss = criterion(xnext, data[1:])
            if epoch_num % 10 == 0 and cnt == 0:
                print(xnext, "\n", data[1:])

            # sloss = 0
            # for x, ugt, u in zip(data, target, u_pred):
            #     res_gt = system(x.unsqueeze(1), ugt.unsqueeze(1), SAMPLE_RATE)
            #     res_pred = system(x.unsqueeze(1), u.unsqueeze(1), SAMPLE_RATE)
            #     sloss += criterion(res_gt, res_pred)
            # loss = sloss / data.size(0)

            mean_loss += loss
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            cnt += 1
        # break
        scheduler.step()
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
    # for batch_idx, (data, target) in enumerate(test_loader):
    #     data, target = data.cuda(), target.cuda()
    #     print(data)
    #     for _ in range(HORIZON):
    #         u_pred = model(data)
    #         u_pred = u_pred.cuda()
    #         for x, ugt, u in zip(data, target, u_pred):
    #             res_pred = system(x.unsqueeze(1), u.unsqueeze(1), SAMPLE_RATE)
    #             print(res_pred)
    #             data = res_pred.squeeze(1).unsqueeze(0)
    #     break

    for i, (data, target) in enumerate(test_loader):
        data, target = data.cuda(), target.cuda()
        pred = model(data)
        for _ in range(HORIZON):
            print(data)
            res = system(torch.transpose(data, 0, 1), torch.transpose(pred, 0, 1), SAMPLE_RATE)
            res = torch.transpose(res, 0, 1)
            data = res
            pred = model(data)
            pred = torch.clamp(pred, min=u_range[0, :], max=u_range[1, :])
        print("\n")
        if i == 0:
            break

    print("\n\n---------------------------\n")

    for i, (x, y) in enumerate(test_loader):
        print(x)
        if i == HORIZON - 1:
            break


if __name__ == '__main__':
    main()
