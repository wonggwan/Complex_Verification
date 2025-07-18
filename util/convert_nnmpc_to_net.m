function net = convert_nnmpc_to_net(weights, biases, activation, sys)
    net.activation = activation;
 	net.dims = size(weights{1},2); % input dimension
    if isempty(sys)
        for i=1:length(weights)
            net.dims = [net.dims size(weights{i},1)]; % dimension of the i-th layer
            net.weights{i} = weights{i};
            net.biases{i}  = biases{i};
        end
    else
        A = sys.A;
        B = sys.B;
        nx = size(A,1);
        % First layer.
        W_bar = [weights{1}; eye(nx)];
        b_bar = [biases{1}  zeros(1,nx)];
      	net.dims = [net.dims size(W_bar,1)];
        net.weights{1} = W_bar;
        net.biases{1}  = b_bar;
        % Middle layers.
        for i=2:length(weights)-1
            W_bar = blkdiag(weights{i},eye(nx)); % construct augmented weight matrix
            b_bar = [biases{i} zeros(1,nx)];
            net.dims = [net.dims size(W_bar,1)]; % dimension of the i-th layer
            net.weights{i} = W_bar;
            net.biases{i}  = b_bar;
        end
        % Last layer.
        W_bar = [B*weights{end} A];
        b_bar = B*biases{end};
        net.dims = [net.dims size(W_bar,1)];
        net.weights{end+1} = W_bar;
        net.biases{end+1}  = b_bar;
    end
end

