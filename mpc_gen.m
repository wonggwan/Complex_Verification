clc; clear all; close all;
addpath('./util');
addpath('./output');

% Initial state x(0)
X0=[-10;30;(0*pi)/180];
vk=0; %linear velocity
wk=0; %angular velocity

Ts=0.01;
s=10;
Simlength=(s/Ts);      

thetak=-1.2586;

D=zeros(3,1);
N=5;
Xr=[50 -50 0]';


% Define cost functionx| and expected disturbances
Q=[1 0 0;0 1 0;0 0 1000];
R=[1 0;0 0.001];
W=ones(1,N)';  % expected demand (this is just an example)

[A B C]=model_system(vk,thetak,Ts);
[A,B,D,Q]=increase_matrixDUQ(A,B,D,Q);
[Gx,Gu,Gw]=constants_mpc(A,B,D,N);

% Build R_hat
R_hat = kron(eye(N),R);
% Build Q_hat
Q_hat=kron(eye(N),Q);

% Constraints
Ax=[1 0 0;0 1 0;-1 0 0 ;0 -1 0];
bx=100*[150; 150;150; 150];
Au=[1 0;0 1 ;-1 0;0 -1];
bu=[100; 1; 100; 1];


Axaum=[Ax zeros(size(Ax,1),size(Au,2));zeros(size(Au,1),size(Ax,2)) Au];
bxaum=[bx;bu];
Ax=Axaum;
bx=bxaum;
% bu=[10; 1; 10; 1];
% Transform into U constraints
Au_hat=kron(eye(N),Au);
bu_hat=kron(ones(N,1),bu);
Ax_hat=kron(eye(N),Ax);
bx_hat=kron(ones(N,1),bx);
%Delta U


% Aggregated U constraints
AU=[Ax_hat*Gu; Au_hat];
%bU=[bx_hat-Ax_hat*Gx*X0-Ax_hat*Gw*W;bu_hat];

% MPC into action
Xhist=X0';
Uhist=[];
VK=vk;
THK=thetak;
Disturb= normrnd(0.5,1,Simlength+N,1); %Longer than simulation for prediction horizon
% Simulation loop
XR=[];
u=[0; 0];
D=zeros(3,1);
path=createPath();
i=1;
delta=0.1;
for k=1:Simlength
    % expected disturbances (force that they are different)
    W=0*Disturb(k:k+N-1)+0*normrnd(0,0.2,N,1); 
    % Update controller matrices for current state and disturbances (H and Au are constant)
    [A B C]=model_system(vk,thetak,Ts);
    [Xr,i]=createReferenceDU(path,i,X0,B,vk,Ts,N,delta);
    UMPC=MPC_DU(A,B,D,N,W,X0,Xr,Q_hat,R_hat,Au_hat,bu_hat,Ax_hat,bx_hat,u);
    XR=[XR Xr(1:3)];
    % Apply only first component
    u=UMPC(1:size(B,2))+u;
    X1=nonlinearModel(D,u,Disturb(k),X0,thetak,wk,vk,Ts);

    vk=saturated(-10,20,vk+u(1));
    wk=saturated(-1,1,wk+u(2));
    thetak=X1(3);
    X0=X1;
    VK=[VK vk];
    THK=[THK thetak];
    Xhist=[Xhist; X0'];
    Uhist=[Uhist; u'];
    X0
    u
    reachability = rank([B A*B]);
    if reachability == 3
        disp("reachable")
    end
end

size(Xhist)
X_train_ri = Xhist;
y_train_ri = Uhist;

% remember to change name of config before you setup
save './output/mpc_x.mat' X_train_ri
save './output/mpc_y.mat' y_train_ri

load mpc_x
load mpc_y

plot(X_train_ri(:,1),X_train_ri(:,2),'ro','LineWidth',1.5)
grid on
axis equal
