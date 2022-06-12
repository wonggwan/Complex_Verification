%% Reach-SDP: 6D Quadrotor Example
function [is_satisfied, res_q, res_Q, success_iter] = rsdp_6d(Ac, Bc, Ec, g, ts, q0, Q0, ...
                                                          avoid_set_xyz, goal_set_xyz, cur_controller,...
                                                          process_ind, sdp_iter)
%% System Parameters
addpath('./util')
addpath('./output');
addpath('C:/Program Files/Mosek/9.3/toolbox/R2015a');
addpath('C:/Program Files/Mosek/9.3/toolbox/R2015aom');

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
load(cur_controller)
net = convert_nnmpc_to_net(weights, biases, 'relu', []);


%% SDP setup
% Initial state set.
% R = Q0;
% X0_ell = ellipsoid(q0, blkdiag(R(1)^2,R(2)^2,R(3)^2,R(4)^2,R(5)^2,R(6)^2))

X0_ell = ellipsoid(q0, Q0);

repeated = true;
mode = 'optimization';
verbose = false;

% Reachability horizon.
N = sdp_iter;

message = ['Starting FRS computation, N = ', num2str(N)];
disp(message);

%% Monte-Carlo Sampling to Compute Reachable Sets
Xg_cell = {}; % Grid-based reachable sets
Ug_cell = {}; % Grid-based control sets
[X0_q,X0_Q] = double(X0_ell);
xQ = sqrtm(X0_Q+eye(6)*1e-6);
[A0,b0] = ell_to_Ab(X0_ell);

A0, b0
Nsample = 20000;
X_sample_box = [X0_q(1)+xQ(1,1)*(1-2*rand(1,Nsample));
                X0_q(2)+xQ(2,2)*(1-2*rand(1,Nsample));
                X0_q(3)+xQ(3,3)*(1-2*rand(1,Nsample));
                X0_q(4)+xQ(4,4)*(1-2*rand(1,Nsample));
                X0_q(5)+xQ(5,5)*(1-2*rand(1,Nsample));
                X0_q(6)+xQ(6,6)*(1-2*rand(1,Nsample))];
Xg = [];
for x = X_sample_box
    if norm(A0*x+b0)<=1
        Xg = [Xg x];
    end
end

Xg_cell{end+1} = Xg;
for i = 1:N
    Xg_t = []; 
    Ug_t = [];
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
    if sol.solver_status == 0
        next_ell = X0_ell;
        [res_q, res_Q] = double(next_ell);
        save next_ell next_ell
        is_satisfied = 0;
        break
    end
    input_set.set = sol.ell;
    ell_seq_vec = [ell_seq_vec sol.ell];
end
ell_length = length(ell_seq_vec);


%% Plots
ell_seq_xyz = [];
Xg_cell_xyz = {};
FRS_xyz = {};

for i = 1:ell_length
    ell_seq_tmp = ell_seq_vec(i);
    ell_seq_xyz = [ell_seq_xyz projection(ell_seq_tmp,...
        [1 0 0 0 0 0; 0 1 0 0 0 0; 0 0 1 0 0 0]')];
    ell_3d = projection(ell_seq_tmp,[1 0 0 0 0 0; 0 1 0 0 0 0; 0 0 1 0 0 0]');
    
    [eq,eQ] = double(ell_3d);
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
    Xg_cell_xyz{end+1} = [Xg_tmp(:,1) Xg_tmp(:,2) Xg_tmp(:,3)];
end

avoid_set_cell = {};
if ~isempty(avoid_set_xyz)
    for i = 1: height(avoid_set_xyz)
        aset = avoid_set_xyz(i,:);
        avoid_set = create_3d_shape(aset(1),aset(2),aset(3),aset(4),aset(5),aset(6));
        avoid_set_cell{end+1} = avoid_set;
    end
end

gset = goal_set_xyz;
goal_set = create_3d_shape(gset(1),gset(2),gset(3),gset(4),gset(5),gset(6));
[is_satisfied, res_index] = quad_verify_plot(ell_seq_xyz, FRS_xyz,...
                            Xg_cell_xyz, avoid_set_cell, goal_set, process_ind);
                        
xlabel('p_x')
ylabel('p_y')
zlabel('p_z')

if is_satisfied
    next_ell = ell_seq_vec(res_index);
    success_iter = res_index;
    next_ell
else
    next_ell = X0_ell;
    success_iter = -1;
    next_ell
end
[res_q, res_Q] = double(next_ell);
save next_ell next_ell
return 
end

%% Functions
function out = proj(u, u_min, u_max)
    out = min(max(u,u_min),u_max);
end