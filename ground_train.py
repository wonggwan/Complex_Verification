import torch
import torch.nn as nn
import torch.optim as optim
from scipy.io import savemat
import numpy as np
from tqdm import tqdm
from util.train_helper import Network, extract_weights, create_ground_data_loaders, ground_dym, ground_dym_test

INPUT_SIZE = 3
OUTPUT_SIZE = 2
HORIZON = 12
BATCH_SIZE = HORIZON
NUM_EPOCHS = 120
LEARNING_RATE = 0.01
SAMPLE_RATE = 0.1  # ts
net_dims = [INPUT_SIZE, 40, 40, OUTPUT_SIZE]
data_folder_name = './output/ground/con1_ts0.1_n12'
u_range = torch.tensor([[-1, -1], [1, 1]]).cuda()


def main():
    train_loader, test_loader = create_ground_data_loaders(BATCH_SIZE, data_folder_name)
    model = Network(net_dims, activation=nn.ReLU).net
    model = model.cuda()
    criterion = nn.MSELoss()
    optimizer = optim.Adam(model.parameters(), lr=LEARNING_RATE)
    scheduler = torch.optim.lr_scheduler.StepLR(optimizer, step_size=30, gamma=0.3)

    for epoch_num in tqdm(range(1, NUM_EPOCHS + 1)):
        mean_loss, cnt = 0, 0
        print("Epoch->{}\n".format(epoch_num))
        model.train()

        for batch_idx, (data, target) in enumerate(train_loader):
            data, target = data.cuda(), target.cuda()
            """
            Train in a simulated 'RNN' fashion
            """
            xprev = data[:BATCH_SIZE - 1]  # 0:Hor-2 data
            pred = model(xprev)
            pred = torch.clamp(pred, min=u_range[0, :], max=u_range[1, :])
            xnext = ground_dym(xprev, pred, SAMPLE_RATE)
            if epoch_num % 10 == 0 and cnt == 0:
                print(xnext, "\n", data[1:])
            loss = criterion(xnext, data[1:])
            mean_loss += loss
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            cnt += 1

        scheduler.step()
        val = mean_loss / cnt
        val = float(val.cpu().detach().numpy())
        print("mean loss->", val, "\n")

    weights, biases = extract_weights(model)
    data = {'weights': np.array(weights, dtype=object), 'biases': np.array(biases, dtype=object)}
    fname = './output/ground/ground_mpc.mat'
    savemat(fname, data)
    save_path = './output/ground/ground_mpc_py.pth'
    torch.save(model.state_dict(), save_path)
    model.eval()
    model.cpu()
    test_u_range = u_range.cpu()

    for i, (data, target) in enumerate(test_loader):
        pred = model(data)
        pred = torch.clamp(pred, min=test_u_range[0, :], max=test_u_range[1, :])
        for _ in range(HORIZON):
            print(data)
            res = ground_dym_test(data, pred, SAMPLE_RATE)
            data = res
            pred = model(data)
            pred = torch.clamp(pred, min=test_u_range[0, :], max=test_u_range[1, :])
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
