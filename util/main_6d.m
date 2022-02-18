%% Reach-SDP: 6D Quadrotor Example

%% System Parameters
clear all; close all; clc
addpath('./util')
addpath('./output');
addpath('C:/Program Files/Mosek/9.3/toolbox/R2015a');
addpath('C:/Program Files/Mosek/9.3/toolbox/R2015aom');
% mkdir ./output/Figure

%% 6D quadrotor model 
ts = 0.3;
% Reachability iteration numbers
N = 6;

g = 9.81;
Ac = [0 0 0 1 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1; zeros(3,6)];
Bc = [zeros(3,3); g 0 0; 0 -g 0; 0 0 1]; % u = [tan(theta) tan(phi) tau]
Ec = [0; 0; 0; 0; 0; -1];

sys_c = ss(Ac,Bc,eye(6),[]);
% DT-system: x_next = Ax + Bu + Eg
sys_d = c2d(sys_c,ts);
A = sys_d.A;
B = sys_d.B;
syms tau
E = double(int(expm(Ac*(ts-tau))*Ec,tau,0,ts));

C = eye(2);
n = size(B,1);
m = size(B,2);
sys.A = A;
sys.B = B;
sys.E = E;
sys.e = g;
sys.ulb = [-pi/9;-pi/9;0];
sys.uub = [ pi/9; pi/9;2*g];

load quad_mpc_6


net = convert_nnmpc_to_net(weights, biases, 'relu', []);


%% SDP setup
% Initial state set.
R = [0.05, 0.05, 0.05, 0.01, 0.01, 0.01];
X0_ell  = ellipsoid( [3.1; 3.5; 3.1; -0.5; -0.5; -0.5],...
    blkdiag(R(1)^2,R(2)^2,R(3)^2,R(4)^2,R(5)^2,R(6)^2) );

avoid_set = create_3d_shape(1.05,1.06,1.05,1.06,1.05,1.06);
goal_set = create_3d_shape(1.5,2.5,3.5,4.5,1.5,2.5);

%% Gaussian distribution
% probably not that useful now
X0_3d = projection(X0_ell,[1 0 0 0 0 0; 0 1 0 0 0 0; 0 0 1 0 0 0]');
[iq,iQ] = double(X0_3d);
iQ = sqrtm(iQ+eye(3)*1e-6);
step = (iq(1)-iQ(1,1))/100;
mu = iq;
Sigma = iQ;

Nsample = 200;
X_sample_box = [mu(1)+Sigma(1,1)*(1-2*rand(1,Nsample));
                mu(2)+Sigma(2,2)*(1-2*rand(1,Nsample));
                mu(3)+Sigma(3,3)*(1-2*rand(1,Nsample))];
FRS_xyz_tmp = [];
for x = X_sample_box
    val1 = x-mu;
    val2 = inv(Sigma)*(x-mu);
    if dot(val1, val2) <= 1
        s = [x(1); x(2); x(3)];
        FRS_xyz_tmp = [FRS_xyz_tmp s];
    end
end
% plot3(FRS_xyz_tmp(1,:),FRS_xyz_tmp(2,:), FRS_xyz_tmp(3,:))
% For x~N(m,p), y = Ax + b -> y~N(Am+b, APA^T)
% For x_next = Ax + Bu + Eg

% return 

%% remaining configs
repeated = true;
mode = 'optimization';
verbose = false;
message = ['Starting FRS computation, N = ', num2str(N)];
disp(message);


%% Monte-Carlo Sampling to Compute Reachable Sets
Xg_cell = {}; % Grid-based reachable sets
Ug_cell = {}; % Grid-based control sets
[X0_q,X0_Q] = double(X0_ell);
[A0,b0] = ell_to_Ab(X0_ell);
Nsample = 20000;
X_sample_box = [X0_q(1)+R(1)*(1-2*rand(1,Nsample));
                X0_q(2)+R(2)*(1-2*rand(1,Nsample));
                X0_q(3)+R(3)*(1-2*rand(1,Nsample));
                X0_q(4)+R(4)*(1-2*rand(1,Nsample));
                X0_q(5)+R(5)*(1-2*rand(1,Nsample));
                X0_q(6)+R(6)*(1-2*rand(1,Nsample))];
Xg = [];
for x = X_sample_box
    if norm(A0*x+b0)<=1
        Xg = [Xg x];
    end
end

%% prev
Xg_cell{end+1} = Xg;
for i = 1:N
    Xg_t = []; % One-step FRS at time t
    Ug_t = []; % One-step control set at time t
    for x = Xg_cell{end}
        u = fwd_prop(net,x);
        % x_next = A*x + B*u + E*g;
        x_next = A*x + B*proj(u,sys.ulb,sys.uub) + E*g;
        Xg_t = [Xg_t x_next];
        Ug_t = [Ug_t u];
    end
    Xg_cell{end+1} = Xg_t;
    Ug_cell{end+1} = Ug_t;
end


%% Call Reach-SDP
disp('-------- Sequential SDP --------');
ell_seq_vec = X0_ell;
sdp_type = '';

input_set.type = 'ellipsoid';
input_set.set = X0_ell;

for i = 1:N
    message = ['Sequential SDP Progress: N = ', num2str(i)];
    disp(message);
    sol = reach_sdp_ellipsoid(net,input_set,repeated,verbose,sys);
    if sol.solver_status == 0
        disp('Calculated Reach-SDP has asymmetric ellipsoid shape')
        next_ell = X0_ell;
        save next_ell next_ell
        is_satisfied = 0;
        break
    end
    input_set.set = sol.ell;
    ell_seq_vec = [ell_seq_vec sol.ell];
end
ell_length = length(ell_seq_vec);

for i = 1:ell_length
    ell_seq_vec(i);
end

%% Plots
ell_seq_xy = [];
ell_seq_xyz = [];
Xg_cell_xy = {};
FRS_xyz = {};
Xg_cell_xyz = {};
R1 = [0.1, 0.1, 0.1, 0.01, 0.01, 0.01];
for i = 1:ell_length
    ell_seq_tmp = ell_seq_vec(i); 
    ell_seq_xy = [ell_seq_xy projection(ell_seq_tmp,...
        [1 0 0 0 0 0; 0 1 0 0 0 0]')];
    ell_seq_xyz = [ell_seq_xyz projection(ell_seq_tmp,...
        [1 0 0 0 0 0; 0 1 0 0 0 0; 0 0 1 0 0 0]')];
    ell_3d = projection(ell_seq_tmp,[1 0 0 0 0 0; 0 1 0 0 0 0; 0 0 1 0 0 0]');
    
    [eq,eQ] = double(ell_3d);
    
    % volume = sqrt(det(eQ))
    
    Q = sqrtm(eQ+eye(3)*1e-6);
    Nsample = 20000;
    X_sample_box = [eq(1)+Q(1,1)*(1-2*rand(1,Nsample));
                    eq(2)+Q(2,2)*(1-2*rand(1,Nsample));
                    eq(3)+Q(3,3)*(1-2*rand(1,Nsample))];
    FRS_xyz_tmp = [];
    for x = X_sample_box
        val1 = x-eq;
        val2 = inv(eQ)*(x-eq);
        if dot(val1, val2) <= 1
            s = [x(1); x(2); x(3)];
            FRS_xyz_tmp = [FRS_xyz_tmp s];
        end
    end
    
    FRS_xyz_tmp = FRS_xyz_tmp';
    FRS_xyz{end+1} = [FRS_xyz_tmp(:,1) FRS_xyz_tmp(:,2) FRS_xyz_tmp(:,3)];
    
    Xg_tmp = Xg_cell{i};
    Xg_tmp = Xg_tmp';
    Xg_cell_xy{end+1} = [Xg_tmp(:,1) Xg_tmp(:,2)];
    Xg_cell_xyz{end+1} = [Xg_tmp(:,1) Xg_tmp(:,2) Xg_tmp(:,3)];
end

plot_interval = 1;
plot_tube = true;
quad_plot_check(ell_seq_xy, ell_seq_xyz, FRS_xyz, [1 0 0], '-', plot_interval, false,...
             Xg_cell_xyz, avoid_set, goal_set, 1)
xlabel('p_x')
ylabel('p_y')
zlabel('p_z')

%% Functions
function out = proj(u, u_min, u_max)
    out = min(max(u,u_min),u_max);
end