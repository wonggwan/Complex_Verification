import torch
import torch.nn as nn
import torch.optim as optim
from scipy.io import savemat
import numpy as np
from tqdm import tqdm
from util.train_helper import Network, extract_weights, create_ground_data_loaders, ground_dym, ground_dym_test, ground_eval

INPUT_SIZE = 3
OUTPUT_SIZE = 2
HORIZON = 15
BATCH_SIZE = HORIZON
NUM_EPOCHS = 500
LEARNING_RATE = 0.002
SAMPLE_RATE = 1  # ts
net_dims = [INPUT_SIZE, 40, 40, OUTPUT_SIZE]
data_folder_name = './output/ground/data0_ts_1_n15'
u_range = torch.tensor([[-0.22, -0.15], [0.22, 0.15]]).cuda()


def main():
    train_loader, test_loader = create_ground_data_loaders(BATCH_SIZE, data_folder_name)
    model = Network(net_dims, activation=nn.ReLU).net
    model = model.cuda()
    criterion = nn.MSELoss()
    optimizer = optim.Adam(model.parameters(), lr=LEARNING_RATE)
    # scheduler = torch.optim.lr_scheduler.StepLR(optimizer, step_size=10, gamma=0.9)

    for epoch_num in tqdm(range(1, NUM_EPOCHS + 1)):
        print("Epoch->{}".format(epoch_num))
        model.train()
        mean_loss, cnt = 0, 0
        for batch_idx, (data, target) in enumerate(train_loader):
            data, target = data.cuda(), target.cuda()
            """
            Train in a simulated 'RNN' fashion
            """
            xprev = data[:BATCH_SIZE - 1]  # 0:Hor-2 data
            pred = model(xprev)
            # pred = torch.clamp(pred, min=u_range[0, :], max=u_range[1, :])
            # print(pred)
            xnext = ground_dym(xprev, pred, SAMPLE_RATE)
            # print(xnext, "\n", data[1:])
            loss = criterion(xnext, data[1:])
            # print(xnext, data[1:])
            mean_loss += loss
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            cnt += 1

        # scheduler.step()
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

    # work_space_min = np.array([2.95, 2.95, 3.96])
    # work_space_max = np.array([3.05, 3.05, 3.98])
    work_space_min = np.array([0.95, 0.95, 3.06])
    work_space_max = np.array([1.05, 1.05, 3.10])
    work_range = np.array([work_space_min, work_space_max])
    x0 = np.random.uniform(low=work_range[0, :], high=work_range[1, :], size=3)
    cur_x = torch.Tensor(x0)
    pred = model(cur_x)
    pred = torch.clamp(pred, min=test_u_range[0, :], max=test_u_range[1, :])
    for _ in range(HORIZON):
        print(cur_x, "    ", pred)
        res = ground_eval(cur_x, pred, SAMPLE_RATE)
        cur_x = res
        pred = model(cur_x)
        pred = torch.clamp(pred, min=test_u_range[0, :], max=test_u_range[1, :])



    # for i, (data, target) in enumerate(test_loader):
    #     pred = model(data)
    #     pred = torch.clamp(pred, min=test_u_range[0, :], max=test_u_range[1, :])
    #     for _ in range(HORIZON):
    #         print(data)
    #         res = ground_dym_test(data, pred, SAMPLE_RATE)
    #         data = res
    #         pred = model(data)
    #         pred = torch.clamp(pred, min=test_u_range[0, :], max=test_u_range[1, :])
    #     print("\n")
    #     if i == 0:
    #         break

    print("\n\n---------------------------\n")

    # for i, (x, y) in enumerate(test_loader):
    #     print(x)
    #     if i == HORIZON - 1:
    #         break


if __name__ == '__main__':
    main()
