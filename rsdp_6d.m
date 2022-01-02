%% Reach-SDP: 6D Quadrotor Example
function is_satisfied = rsdp_6d(Ac, Bc, Ec, g, ts, q0, Q0, ...
                                avoid_set_xyz, goal_set_xyz, cur_controller,...
                                is_init)
%% System Parameters
addpath('./util')
addpath('./output');
addpath('C:/Program Files/Mosek/9.3/toolbox/R2015a');
addpath('C:/Program Files/Mosek/9.3/toolbox/R2015aom');

% 6D quadrotor model (ts = 0.1).
%g = 9.81;
%Ac = [0 0 0 1 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1; zeros(3,6)];
%Bc = [zeros(3,3); g 0 0; 0 -g 0; 0 0 1]; % u = [tan(theta) tan(phi) tau]
%Ec = [0; 0; 0; 0; 0; -1];
%ts = 0.1;

sys_c = ss(Ac,Bc,eye(6),[]);
sys_d = c2d(sys_c,ts);
A = sys_d.A;
B = sys_d.B;
syms tau
Bd = int(expm(Ac*(ts-tau))*Bc,tau,0,ts);
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

% Get network parameters
%load nnmpc_nets_quad6d
%load quad_mpc
load(cur_controller)

net = convert_nnmpc_to_net(weights, biases, 'relu', []);


%% SDP setup
% Initial state set.
% R = [0.05, 0.05, 0.05, 0.01, 0.01, 0.01];

% First time
R = Q0;

if is_init
    X0_ell = ellipsoid(q0, blkdiag(R(1)^2,R(2)^2,R(3)^2,R(4)^2,R(5)^2,R(6)^2));
else
    load next_ell
    X0_ell = next_ell;
end

repeated = true;
mode = 'optimization';
verbose = false;

% Reachability horizon.
N = 10;

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

Xg_cell{end+1} = Xg;
for i = 1:N
    Xg_t = []; % One-step FRS at time t
    Ug_t = []; % One-step control set at time t
    for x = Xg_cell{end}
        u = fwd_prop(net,x);
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
    input_set.set = sol.ell;
    ell_seq_vec = [ell_seq_vec sol.ell];
end


%% Plots
ell_seq_xy = [];
Xg_cell_xy = {};
Xg_cell_xyz = {};
for i = 1:N
    ell_seq_tmp = ell_seq_vec(i);
    ell_seq_xy = [ell_seq_xy projection(ell_seq_tmp,...
        [1 0 0 0 0 0; 0 1 0 0 0 0]')];
    Xg_tmp = Xg_cell{i};
    Xg_tmp = Xg_tmp';
    Xg_cell_xy{end+1} = [Xg_tmp(:,1) Xg_tmp(:,2)];
    Xg_cell_xyz{end+1} = [Xg_tmp(:,1) Xg_tmp(:,2) Xg_tmp(:,3)];
end

aset = avoid_set_xyz;
gset = goal_set_xyz;

avoid_set = create_3d_shape(aset(1),aset(2),aset(3),aset(4),aset(5),aset(6));
goal_set = create_3d_shape(gset(1),gset(2),gset(3),gset(4),gset(5),gset(6));

plot_interval = 1;
plot_tube = true;
[is_satisfied, res_index] = plotandcheck(ell_seq_xy, [1 0 0], '-', plot_interval, false,...
                            Xg_cell_xy, Xg_cell_xyz, avoid_set, goal_set);
                        
xlabel('p_x')
ylabel('p_y')
zlabel('p_z')

if is_satisfied
    next_ell = ell_seq_vec(res_index);
else
    next_ell = ell_seq_vec(N); %if not satisfied, return the last one
end
save next_ell next_ell
return 

end

%% Functions
function out = proj(u, u_min, u_max)
    out = min(max(u,u_min),u_max);
end