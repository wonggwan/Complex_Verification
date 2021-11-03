function [is_satisfied, FRS_V_bd] = rsdp(A, B, X0_poly, avoid_x, avoid_y, goal_x, goal_y, controller_name)
%% Reach-SDP with Forward Reachability
addpath('./util');
addpath('./output');
addpath('C:/Program Files/Mosek/9.3/toolbox/R2015a');
addpath('C:/Program Files/Mosek/9.3/toolbox/R2015aom');

%% System Parameters
% double integrator system (ts = 1.0)
% A = [-0.5 0; 0.1 -0.2];
% B = eye(2);

% uub - input upper bound?
% ulb - input lower bound?
sys.uub =  10;
sys.ulb = -10;
% load mnist_weights
load(controller_name)

C = eye(2);
n = size(B,1);
m = size(B,2);
sys.A = A;
sys.B = B;

dim_x = size(sys.A,1);
dim_u = size(sys.B,2);

%% get network parameters

dims(1) = size(weights{1},2);
num_layers = numel(weights)-1;

for i=1:num_layers
    dims(i+1) = size(weights{i},1);
end

dims(num_layers+2) = size(weights{end},1);
net = nnsequential(dims, 'relu');
net.weights = weights;
net.biases = biases;

%% add projection layer
% Projection layer projects output of NN (control action) onto a specified
% set of control inputs
net_p = nnsequential([dims dims(end) dims(end)],'relu');

weights_p = weights;
weights_p{end+1} = -eye(dim_u);
weights_p{end+1} = -eye(dim_u);

biases_p = biases;
biases_p{end} = biases{end}-sys.ulb;
biases_p{end+1} =  sys.uub-sys.ulb;
biases_p{end+1} =  sys.uub;

net_p.weights = weights_p;
net_p.biases = biases_p;

%% Setup
% initial set
% X0_b = [1.5; -1.0; 2.5; -2.0];
% X0_b = [3.0; -2.5; 0.25; 0.25];

% X0_poly = Polyhedron([1 0; -1 0; 0 1; 0 -1], X0_b);
X0_poly = Polyhedron(X0_poly);

X0 = X0_poly.outerApprox; % normalize the A matrix
X0_vec = X0;

% dx = 0.02; % shrink the tube for better visualization
% X0_poly_s = Polyhedron([1 0; -1 0; 0 1; 0 -1], X0_b-dx);
X0_poly_s = X0_poly;

% reachability horizon
N = 6;

% facets of the output polytope
%A_out = [1 0; -1 0; 0 1; 0 -1; -1 1; 1 -1; 1 1; -1 -1; 1 2; -1 -2; -1 2; 1 -2; 2 1; -2 -1; 2 -1; -2 1];
% A_out = [1 0; -1 0; 0 1; 0 -1; -1 1; 1 -1; 1 1; -1 -1; 1 2; -1 -2; 1 4; -1 -4; -1 2; 1 -2; -1 4; 1 -4];
A_out = [1 0; -1 0; 0 1; 0 -1; 1 -1; -1 1; 1 1; -1 -1];
disp(['Starting FRS computation, N = ', num2str(N)]);


%% Gridding to Compute the Exact Reachable Sets
Xg_cell = {}; % grid-based reachable sets
Ug_cell = {}; % grid-based control sets
Xg = grid(X0_poly_s,40);
Xg_cell{end+1} = Xg;
Xg_cell;
for k = 1:N
    Xg_k = []; % one-step FRS at time k
    Ug_k = []; % one-step control set at time k
    for x = Xg_cell{end}'
        u = net.eval(x);
        x_next = A*x + B*u;
        x = x_next;
        Xg_k = [Xg_k; x_next'];
        Ug_k = [Ug_k; u(1)];

    end
    Xg_cell{end+1} = Xg_k;
    Ug_cell{end+1} = Ug_k;
end

%% Reach-SDP
poly_cell = cell(1,N+1);
poly_cell{1,1}  = X0_vec;

options.verbose = 0;
options.solver = 'mosek';
options.repeated = 0;


for i = 1:length(X0_vec)
    % polytopic initial set
    input_set  = X0_vec(i);
    poly_seq_vec   = X0_vec(i);

    % forward reachability
    for k = 1:N
        [b_out,~,~] =  reach_sdp(net,sys.A,sys.B,input_set.H,A_out',options);
        % shift horizon
        input_set = Polyhedron(A_out, b_out);
        % save results
        poly_seq_vec = [poly_seq_vec Polyhedron(A_out, b_out)];
        % report
        disp(['Reach-SDP Progress: N = ', num2str(k), ', i = ',...
            num2str(i), ', volume: ', num2str(input_set.volume)]);
    end
    poly_cell{1,i} = poly_seq_vec;
end


%% Plot results
figure('Renderer', 'painters')
hold on
% initial set
plot(X0_poly,'color','k','alpha',0.1)
% N-step FRS (Forward Reachable Set)
for i = 1:length(X0_vec)
    for k = 1:N+1
        FRS_V = poly_cell{1,i}(k).V;
        FRS_V_bd = FRS_V(boundary(FRS_V(:,1), FRS_V(:,2), 0.0),:);
        plot(polyshape(FRS_V_bd))
        if k == 1
             %continue
             plot(FRS_V_bd(:,1),FRS_V_bd(:,2),'k-','LineWidth',2)
        else
            %continue
            plot(FRS_V_bd(:,1),FRS_V_bd(:,2),'r-','LineWidth',3)
        end
    end

end

%% Goal and Avoiding Set Verification
% gridding states
for k = 2:N+1
    FRS = Xg_cell{k};
    FRS_bd = FRS(boundary(FRS(:,1), FRS(:,2), 0.0),:);
    plot(FRS_bd(:,1),FRS_bd(:,2),'b-','LineWidth',1.5)
end

% Checking if the aviod set is violated during the procedure
% also check if final goal is reached
%avoid_x = [0.5 0.5 1 1];
%avoid_y = [-0.4 -0.8 -0.8 -0.4];
%goal_x= [0.95 1.05 1.05 0.95];
%goal_y= [0.98 0.98 1.02 1.02];

avoid_set = polyshape(avoid_x,avoid_y);
plot(avoid_set);
goal_set = polyshape(goal_x, goal_y);
plot(goal_set);

for i = 1:length(X0_vec)
    for k = 1:N+1
        FRS_V = poly_cell{1,i}(k).V;
        FRS_V_bd = FRS_V(boundary(FRS_V(:,1), FRS_V(:,2), 0.0),:);
        plot(polyshape(FRS_V_bd));
        polyout = intersect(avoid_set, FRS_V_bd);
        if ~isempty(polyout)
            rule_violation = 1;
        else
            rule_violation = 0;
        end
        bd_x = FRS_V_bd(:,1);
        bd_y = FRS_V_bd(:,2);
        [in, on] = inpolygon(bd_x, bd_y, goal_x, goal_y);
    end
end

if numel(bd_x(~in)) ~= 0
    goal_violation = 1;
else
    goal_violation = 0;
end

is_satisfied = 0;

if rule_violation == 1 || goal_violation == 1
    disp('Reach-SDP Not Satisfied');
else
    is_satisfied = 1;
    disp('Reach-SDP Satisfied');
    return
end

grid on;


xlabel('$x_1$','Interpreter','latex');
ylabel('$x_2$','Interpreter','latex');

