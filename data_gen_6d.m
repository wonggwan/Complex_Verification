%% Generate I/O data of a MPC controller
clear all; close all; clc
addpath('./util');
addpath('./output');
setenv('SNOPT_LICENSE','D:/Software/snopt7_matlab/snopt7.lic');

%% Model and specifications ts = 0.5
g = 9.81;
Ac = [0 0 0 1 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1; zeros(3,6)];
Bc = [zeros(3,3); g 0 0; 0 -g 0; 0 0 1]; % u = [tan(theta) tan(phi) tau]
Ec = [0; 0; 0; 0; 0; -1];
ts = 0.5;
sys_c = ss(Ac,Bc,eye(6),[]);
% DT-system: x_next = Ax + Bu + Eg
sys_d = c2d(sys_c,ts);
A = sys_d.A;
B = sys_d.B;
% E = [0; 0; -0.1250; 0; 0;-0.5000]; % ts = 0.5
syms tau
E = double(int(expm(Ac*(ts-tau))*Ec,tau,0,ts));

%% MODEL Specification ts=0.1
% g = 9.81;
% Ac = [0 0 0 1 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1; zeros(3,6)];
% Bc = [zeros(3,3); g 0 0; 0 -g 0; 0 0 1]; % u = [tan(theta) tan(phi) tau]
% Ec = [0; 0; 0; 0; 0; -1];
% ts = 0.1;
% sys_c = ss(Ac,Bc,eye(6),[]);
% sys_d = c2d(sys_c,ts);
% A = sys_d.A;
% B = sys_d.B;
% syms tau
% Bd = int(expm(Ac*(ts-tau))*Bc,tau,0,ts);
% E = double(int(expm(Ac*(ts-tau))*Ec,tau,0,ts));

N = 10; % changed from 30
Q = eye(6);
R = eye(3);
P = eye(6);

xmax = [ 5; 5; 5; 1; 1; 1];
xmin = -xmax;
umin = [-pi/9;-pi/9;0];
umax = [ pi/9; pi/9;2*g];

%% Generate data set
X = [];
y = [];

Xnmpc = [4;4;4;0;0;0];
count = 0;
%% Decrease horizon each time MPC is applied
% for k = 1:1
%     k
%     success_counter = 0;
%     x_tmp = [];
%     y_tmp = [];
%     Ncurr = N;
%     x0 = [5*(1-2*rand); 5*(1-2*rand); 5*(1-2*rand); 1*(1-2*rand); 1*(1-2*rand); 1*(1-2*rand)]; 
%     % x0 = [1*(1-2*rand); 1*(1-2*rand); 1*(1-2*rand); 1*(1-2*rand); 1*(1-2*rand); 1*(1-2*rand)]; 
%     for j = 1:N
%         Ncurr
%         [feas, xOpt, uOpt, JOpt] = solve_nmpc_quad(ts,Q,R,Ncurr,umin,umax,xmin,xmax,x0, Xnmpc, P);
%         if feas == 1
%             x_tmp = [x_tmp; x0'];
%             y_tmp = [y_tmp; tan(uOpt(1,1)) tan(uOpt(2,1)) uOpt(3,1)];
%             u = [tan(uOpt(1,1)); tan(uOpt(2,1)); uOpt(3,1)];
%             success_counter = success_counter + 1;
%             Ncurr = Ncurr - 1;
%             xNext = one_step_sim(x0,u,A,B,E,g);
%             x0 = xNext;
%             x0
%         end
%     end
%     if success_counter == N
%         disp("new datapoint")
%         x = [x; x_tmp];
%         y = [y; y_tmp];
%     end  
% end

%% another version 
% for k = 1:1
%     x0 = [5*(1-2*rand); 5*(1-2*rand); 5*(1-2*rand); 1*(1-2*rand); 1*(1-2*rand); 1*(1-2*rand)]
%     for j = 1:1000
%         [feas, xOpt, uOpt, JOpt] = solve_nmpc_quad(ts,Q,R,N,umin,umax,xmin,xmax,x0, Xnmpc, P);
%         if feas == 1
%             u = [tan(uOpt(1,1)); tan(uOpt(2,1)); uOpt(3,1)];
%             xNext = one_step_sim(x0,u,A,B,E,g);
%             x0 = xNext;
%             x0
%         end
%     end
%     
% end

%% test version
% for k = 1:1200
%     k
%     x0 = [5*(1-2*rand); 5*(1-2*rand); 5*(1-2*rand); 1*(1-2*rand); 1*(1-2*rand); 1*(1-2*rand)]; % randomly select initial points
%     [feas, xOpt, uOpt, JOpt] = solve_nmpc_quad(ts,Q,R,N,umin,umax,xmin,xmax,x0, Xnmpc, P);
%     if feas == 1
%         count = count + 1;
%         for j = 1:N
%             X = [X; x0'];
%             y = [y; tan(uOpt(1,j)) tan(uOpt(2,j)) uOpt(3,j)];
%             u = [tan(uOpt(1,j)); tan(uOpt(2,j)); uOpt(3,j)];
%             xNext = one_step_sim(x0,u,A,B,E,g);
%             x0 = xNext;
%         end
%         % X = [X; x0'];
%         % y = [y; 0 0 0];
%         % stop moving when it goes to the stable point
%     end
% end

%% another test version (from Haimin's original code)
% for k = 1:1
%     k
%     x0 = [5*(1-2*rand); 5*(1-2*rand); 5*(1-2*rand); 1*(1-2*rand); 1*(1-2*rand); 1*(1-2*rand)];
%     [feas, xOpt, uOpt, JOpt] = solve_nmpc_quad(ts,Q,R,N,umin,umax,xmin,xmax,x0, Xnmpc, P);
%     if feas == 1
%         count = count + 1;
%         X = [X; x0'];
%         y = [y; tan(uOpt(1,1)) tan(uOpt(2,1)) uOpt(3,1)];
%     end   
% end
% count
% 
% 
% X_train_nnmpc = X;
% y_train_nnmpc = y;
% save quad_mpc_x X_train_nnmpc
% save quad_mpc_y y_train_nnmpc

load quad_mpc_x_444000
load quad_mpc_y_444000

%% with model SOL
for k = 1:1
    k
    x = X_train_nnmpc(k,:)'
    % x = [2;2;2;1;0;0]
    [feas, xOpt, uOpt, JOpt] = solve_nmpc_quad(ts,Q,R,N,umin,umax,xmin, xmax,x, Xnmpc, P);
    if feas == 1
        for j = 1:10
            u = [tan(uOpt(1,j)); tan(uOpt(2,j)); uOpt(3,j)];
            xNext = one_step_sim(x,u,A,B,E,g);
            x = xNext;
            x
        end
        
    end
end

%% with model SOL another case
% N = 10;
% for k = 1:1
%     k
%     x = X_train_nnmpc(k,:)'
%     x_b = x;
%     for j = 1:100
%         [feas, xOpt, uOpt, JOpt] = solve_nmpc_quad(ts,Q,R,N,umin,umax,xmin, xmax,x, Xnmpc, P);
%         if feas == 1
%             u = [tan(uOpt(1,1)); tan(uOpt(2,1)); uOpt(3,1)];
%             xNext = one_step_sim(x,u,A,B,E,g);
%             x = xNext;
%             x
%         end
%     end
    
    
%     [feas, xOpt, uOpt, JOpt] = solve_nmpc_quad(ts,Q,R,N,umin,umax,xmin, xmax,x_b, Xnmpc, P);
%     if feas == 1
%         for j = 1:9
%             u = [tan(uOpt(1,j)); tan(uOpt(2,j)); uOpt(3,j)];
%             xNext = one_step_sim(x_b,u,A,B,E,g);
%             x_b = xNext
%         end
%         x_b;
%     end
% end


%% SOL A
% for k = 1:1
%     k
%     x = [5*(1-2*rand); 5*(1-2*rand); 5*(1-2*rand); 1*(1-2*rand); 1*(1-2*rand); 1*(1-2*rand)]
%     [feas, xOpt, uOpt, JOpt] = solve_nmpc_quad(ts,Q,R,N,umin,umax,xmin, xmax,x, Xnmpc, P);
%     if feas == 1
%         for j = 1:30
%             u = [tan(uOpt(1,j)); tan(uOpt(2,j)); uOpt(3,j)];
%             xNext = one_step_sim(x,u,A,B,E,g)
%             x = xNext;
%         end  
%     end
% end

%% SOL B
% for k = 1:1
%     k
%     x = [5*(1-2*rand); 5*(1-2*rand); 5*(1-2*rand); 1*(1-2*rand); 1*(1-2*rand); 1*(1-2*rand)]
%     for j = 1:30
%         [feas, xOpt, uOpt, JOpt] = solve_nmpc_quad(ts,Q,R,N,umin,umax,xmin, xmax,x, Xnmpc, P);
%         if feas == 1
%             x1 = x;
%             for j = 1:30
%                 utmp = [tan(uOpt(1,j)); tan(uOpt(2,j)); uOpt(3,j)];
%                 xtmp = one_step_sim(x1,utmp,A,B,E,g)
%                 x1 = xtmp;
%             end  
%             u = [tan(uOpt(1,1)); tan(uOpt(2,1)); uOpt(3,1)];
%             xNext = one_step_sim(x,u,A,B,E,g);
%             x = xNext;
%         end  
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

%% Attribution
% Haimin Hu, March 2020