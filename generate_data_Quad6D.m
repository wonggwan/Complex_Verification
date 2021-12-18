%% Generate I/O data of a MPC controller
clear all; close all; clc
setenv('SNOPT_LICENSE','D:/Software/snopt7_matlab/snopt7.lic');

%% Model and specifications
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
% syms tau
% Bd = int(expm(Ac*(ts-tau))*Bc,tau,0,ts);
% E = int(expm(Ac*(ts-tau))*Ec,tau,0,ts);
% E = [0; 0; -0.0050; 0; 0;-0.1000]; % ts = 0.1
E = [0; 0; -0.1250; 0; 0;-0.5000]; % ts = 0.5
N = 30;
Q = eye(6);
R = eye(3);

xmax = [ 5; 5; 5; 1; 1; 1];
xmin = -xmax;
umin = [-pi/9;-pi/9;0];
umax = [ pi/9; pi/9;2*g];

E_obs = ellipsoid([1;1;1],blkdiag(1.5^2,1.5^2,1.5^2));

%% Generate data set
X = [];
y = [];
for k = 1:4000
    k
    x0 = [5*(1-2*rand); 5*(1-2*rand); 5*(1-2*rand); 1*(1-2*rand); 1*(1-2*rand); 1*(1-2*rand)]; % randomly select initial points
    %x0 = [(1-2*rand)/10;(1-2*rand)/10;(1-2*rand)/10; 1*(1-2*rand)/10; 1*(1-2*rand)/10; 1*(1-2*rand)]/10;
    
    [feas, xOpt, uOpt, JOpt] = solve_nmpc_quad(ts,Q,R,N,umin,umax,xmin,xmax,x0,E_obs);
    if feas == 1
        X = [X; x0'];
        y = [y; tan(uOpt(1,1)) tan(uOpt(2,1)) uOpt(3,1)];
    end
end

X_train_nnmpc = X;
y_train_nnmpc = y;
save X_train_nnmpc X_train_nnmpc
save y_train_nnmpc y_train_nnmpc

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
% x = [px, py, pz, vx, vy, vz]
% u = [theta, phi, tau]
    g = 9.81;
    dxdt = [x(4); x(5); x(6); g*tan(u(1)); -g*tan(u(2)); u(3)-g];
end


%% Attribution
% Haimin Hu, March 2020