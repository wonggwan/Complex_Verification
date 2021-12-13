clc; clear all; close all;
addpath('./util');
addpath('./output');

% Initial state x(0)
X0=[0;0;(0*pi)/180];
vk=0;
thetak=-1.2586;
vk=0;
Ts=0.001;
D=zeros(3,1);
N=10;
Xr=[100 101 0]';
%Xr(3)=(atan((-50-X0(2))/(-50-X0(1))));

% % Define cost function and expected disturbances
% Q=eye(3);
% R=eye(2);
% W=ones(1,N)';  % expected demand (this is just an example)

% Define cost functionx| and expected disturbances
Q=[0.00001 0 0;0 1 0;0 0 100000000];
R=[0.01 0;0 100];
W=ones(1,N)';  % expected demand (this is just an example)

[A B C]=model_system(vk,thetak,Ts);
[Gx,Gu,Gw]=constants_mpc(A,B,D,N);
% Build R_hat
R_hat = kron(eye(N),R);
% Build Q_hat
Q_hat=kron(eye(N),Q);


% Constraints
Ax=[1 0 0;0 1 0;-1 0 0 ;0 -1 0];
bx=[50; 50;50; 50];
Au=[1 0;0 1;-1 0;0 -1];
bu=[150; 1; 25; 1];

% Transform into U constraints
Au_hat=kron(eye(N),Au);
bu_hat=kron(ones(N,1),bu);
Ax_hat=kron(eye(N),Ax);
bx_hat=kron(ones(N,1),bx);

% Aggregated U constraints
AU=[Ax_hat*Gu; Au_hat];
bU=[bx_hat-Ax_hat*Gx*X0-Ax_hat*Gw*W;bu_hat];

% MPC into action
Simlength=10000;
Xhist=[];
Uhist=[];
Disturb= normrnd(0.5,1,Simlength+N,1); %Longer than simulation for prediction horizon
for k=1:Simlength
    
    % expected disturbances (force that they are different)
    W=Disturb(k:k+N-1)+0*normrnd(0,0.2,N,1); 
    
    % Update controller matrices for current state and disturbances (H and Au are constant)
    [A B C]=model_system(vk,thetak,Ts);
    [Gx,Gu,Gw]=constants_mpc(A,B,D,N);

    % Build cost function
    H=Gu'*Q_hat*Gu+R_hat;
    F=X0'*Gx'*Q_hat*Gu+W'*Gw'*Q_hat*Gu-kron(ones(N,1),Xr)'*Q_hat*Gu;
    UMPC=quadprog(H,F,AU,bU);

    % Apply only first component
    u=UMPC(1:size(B,2));
    X1=A*X0+B*u+D*Disturb(k);
    dx=(X1(1)-X0(1))/Ts;
    dy=(X1(2)-X0(2))/Ts;

    X0=X1;
    Xhist=[Xhist; X0'];
    Uhist=[Uhist; u'];
    
    X0

end

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

