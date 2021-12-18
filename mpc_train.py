import torch
import torch.nn as nn
import torch.optim as optim
from scipy.io import savemat
import numpy as np
from util.train_helper import Network, extract_weights, create_mpc_data_loaders

INPUT_SIZE = 3
OUTPUT_SIZE = 2
BATCH_SIZE = 100
NUM_EPOCHS = 100
LEARNING_RATE = 0.001


def main():
    train_loader, test_loader = create_mpc_data_loaders(BATCH_SIZE)
    net_dims = [INPUT_SIZE, 20, 10, 5, OUTPUT_SIZE]
    model = Network(net_dims, activation=nn.ReLU).net
    model = model.cuda()

    criterion = nn.L1Loss(reduction='mean')
    optimizer = optim.Adam(model.parameters(), lr=LEARNING_RATE)

    mean_loss, cnt = 0, 0
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

    weights, biases = extract_weights(model)
    data = {'weights': np.array(weights, dtype=np.object), 'biases': np.array(biases, dtype=np.object)}

    fname = './output/quad_mpc.mat'
    savemat(fname, data)


if __name__ == '__main__':
    main()
