%% Generate I/O data of a MPC controller
clear all; close all; clc
addpath('./util');
addpath('./output');
setenv('SNOPT_LICENSE','D:/Software/snopt7_matlab/lib/snopt7.lic');

%% important Varables
ts = 0.3; % samping rate
N = 10; % horizon
goal_max = [0.5;0.5;0.5]; % max value of goal room
goal_min = [-0.5;-0.5;-0.5]; % min value of goal room
goal_speed = [0; 0; 0];

%% system dynamics
g = 9.81;
Ac = [0 0 0 1 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1; zeros(3,6)];
Bc = [zeros(3,3); g 0 0; 0 -g 0; 0 0 1]; % u = [tan(theta) tan(phi) tau]
Ec = [0; 0; 0; 0; 0; -1];
sys_c = ss(Ac,Bc,eye(6),[]);
sys_d = c2d(sys_c,ts);
A = sys_d.A;
B = sys_d.B;
syms tau
Bd = int(expm(Ac*(ts-tau))*Bc,tau,0,ts);
E = double(int(expm(Ac*(ts-tau))*Ec,tau,0,ts));
Q = eye(6);
R = eye(3);
P = eye(6);

%% normal case
xmax = [ 5; 5; 5; 1; 1; 1];
xmin = -xmax;
%% correct u limits
umin = [-pi/9;-pi/9;0];
umax = [ pi/9; pi/9;2*g];
%% Generate data set
X = [];
y = [];
count = 0;

%% TEST WITH OBSTALCES CHECKING
k = 0;
while k < 800
    k
    % select some out of distribution datapoints to enlarge the coverage of the dataset volume
    x0 = [5*(1-2*rand); 5*(1-2*rand); 5*(1-2*rand); 1*(1-2*rand); 1*(1-2*rand); 1*(1-2*rand)];
    if x0(1)<xmin(1)|| x0(1)>xmax(1) || x0(2)<xmin(2) || x0(2)>xmax(2) || x0(3)<xmin(3) || x0(3)>xmax(3)
        continue %reject this initial state
    end
    x0
    [feas, xOpt, uOpt, JOpt] = solve_quadmpc(ts,Q,R,N,umin,umax,xmin,xmax,x0,P,...
                                             goal_max,goal_min,goal_speed);
    if feas == 1
        count = count + 1;
        for j = 1:N
            u = [tan(uOpt(1,j)); tan(uOpt(2,j)); uOpt(3,j)];
            xNext = one_step_sim(x0,u,A,B,E,g);
            x0 = xNext;
            
            X = [X; x0'];
            y = [y; tan(uOpt(1,j)) tan(uOpt(2,j)) uOpt(3,j)];
        end
        k = k + 1;
    end
end

X_train_nnmpc = X;
y_train_nnmpc = y;
save quad_mpc_x X_train_nnmpc
save quad_mpc_y y_train_nnmpc


%% with model SOL
% load quad_mpc_x
% load quad_mpc_y
% x = X_train_nnmpc(1,:)'
% [feas, xOpt, uOpt, JOpt] = solve_quadmpc(ts,Q,R,N,umin,umax,xmin,xmax,x,P,...
%                                          goal_max,goal_min,goal_speed);
% if feas == 1
%    for j = 1:N
%         u = [tan(uOpt(1,j)); tan(uOpt(2,j)); uOpt(3,j)];
%         xNext = one_step_sim(x,u,A,B,E,g);
%         x = xNext;
%         x
%     end
% end



%% Plot data
xc=0; yc=0; zc=0;    % coordinated of the center
alpha=0.0;           % transparency
X = [0 0 0 0 0 1; 1 0 1 1 1 1; 1 0 1 1 1 1; 0 0 0 0 0 1];
Y = [0 0 0 0 1 0; 0 1 0 0 1 1; 0 1 1 1 1 1; 0 0 1 1 1 0];
Z = [0 0 1 0 0 0; 0 0 1 0 0 0; 1 1 1 0 1 1; 1 1 1 0 1 1];
C='k';
X = 2*xmax(1)*(X-0.5) + xc;
Y = 2*xmax(2)*(Y-0.5) + yc;
Z = 2*xmax(3)*(Z-0.5) + zc; 
fill3(X,Y,Z,C,'FaceAlpha',alpha);    % draw cube
axis equal
hold on
plot3(X_train_nnmpc(:,1),X_train_nnmpc(:,2),X_train_nnmpc(:,3),'r.','MarkerSize',5)

%% Auxiliary functions
function xNext = one_step_sim(x,u,A,B,E,g)
    xNext = A*x+B*u+E*g;
end

function xNext = RK4(x, u, ts, n)
    dt = ts/n;
    for rki = 1 : n
        k1 = ode_nominal(x, u);
        k2 = ode_nominal(x + (dt/2)*k1, u);
        k3 = ode_nominal(x + (dt/2)*k2, u);
        k4 = ode_nominal(x + dt*k3, u);
        x  = x + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);
    end
    xNext = x;
end

function dxdt = ode_nominal(x, u)
    g = 9.81;
    dxdt = [x(4); x(5); x(6); g*tan(u(1)); -g*tan(u(2)); u(3)-g];
end
