%% Generate I/O data of a MPC controller
clear all; close all; clc
addpath('./util');
addpath('./output');
setenv('SNOPT_LICENSE','D:/Software/snopt7_matlab/lib/snopt7.lic'); 
% license only valid for 3 months
 
%% important Varables
ts = 0.1; % samping rate
N = 12; % horizon

goal_min = [0.5;0.5]; % min value of goal room
goal_max = [1.5;1.5]; % max value of goal room
goal_theta = pi/2;

%% system dynamics
Q = eye(3);
R = eye(2);

%% system state constraint
xmax = [ 5;  5; 2*pi];
xmin = [-5; -5;    0];

%% Control input constraints are for v and w (motion primitives)
umin = [-1;-1];
umax = [ 1; 1];

%% Generate data set
X = [];
y = [];
k = []; % For training purpose
count = 0;

%% TEST WITH OBSTALCES CHECKING
cnt = 0;
while cnt < 600
    cnt
    x0 = [5*(1-2*rand); 5*(1-2*rand); 2*pi*(1-rand)]
    [feas, xOpt, uOpt, JOpt] = solve_ground(ts,Q,R,N,umin,umax,...
                                            xmin,xmax,x0,...
                                            goal_min,goal_max,goal_theta);
    if feas == 1
        count = count + 1;
        xOpt;
        for j = 1:N
            [xNext, k1, k2, k3] = evolve(x0,uOpt(:,j),ts);
            x0 = xNext;
            X = [X; x0'];
            y = [y; uOpt(:,j)'];
            
        end
        cnt = cnt + 1;
    end
end

X_train_nnmpc = X;
y_train_nnmpc = y;
save ground_mpc_x X_train_nnmpc
save ground_mpc_y y_train_nnmpc

scatter(X_train_nnmpc(:,1), X_train_nnmpc(:,2))
hold on
line(X_train_nnmpc(:,1),X_train_nnmpc(:,2))

%% Plot data
% xc=0; yc=0; zc=0;    % coordinated of the center
% alpha=0.0;           % transparency
% X = [0 0 0 0 0 1; 1 0 1 1 1 1; 1 0 1 1 1 1; 0 0 0 0 0 1];
% Y = [0 0 0 0 1 0; 0 1 0 0 1 1; 0 1 1 1 1 1; 0 0 1 1 1 0];
% Z = [0 0 1 0 0 0; 0 0 1 0 0 0; 1 1 1 0 1 1; 1 1 1 0 1 1];
% C='k';
% X = 2*xmax(1)*(X-0.5) + xc;
% Y = 2*xmax(2)*(Y-0.5) + yc;
% Z = 2*xmax(3)*(Z-0.5) + zc; 
% fill3(X,Y,Z,C,'FaceAlpha',alpha);    % draw cube
% axis equal
% hold on
% plot3(X_train_nnmpc(:,1),X_train_nnmpc(:,2),X_train_nnmpc(:,3),'r.','MarkerSize',5)

%% Auxiliary functions
function [res, k1, k2, k3] = evolve(x, u, ts)
    % https://escholarship.org/content/qt1340479b/qt1340479b.pdf
    tmp = u(2)*ts/2 + 1e-6;
    k1 = x(1) + u(1) * sin(tmp*pi)/(tmp*pi) * cos(x(3)+tmp);
    k2 = x(2) + u(1) * sin(tmp*pi)/(tmp*pi) * sin(x(3)+tmp);
    k3 = x(3) + ts * u(2);
    res = [k1; k2; k3];  
end