function [feas, xOpt, uOpt, JOpt] = solve_mpc(A,B,P,Q,R,N,umin,umax,...
    xmin,xmax,Oinf,x0)
% Solve the MPC problem

% Define system dimensions
NX = length(A);
NU = length(B(1,:));
%NU

% Define decision variables
x_yal = sdpvar(NX,N+1);
u_yal = sdpvar(NU,N);

% Define objective, constraints and terminal set
objective = x_yal(:,N+1)'*P*x_yal(:,N+1);
for i = 1:N
    objective = objective + x_yal(:,i)'*Q*x_yal(:,i) +...
        u_yal(:,i)'*R*u_yal(:,i);
end

constraints = x_yal(:,1)==x0;
for i = 1:N
    % state and input
    constraints = [constraints xmin <= x_yal(:,i) <= xmax...
                               umin <= u_yal(:,i) <= umax];
    % dynamics
    constraints = [constraints x_yal(:,i+1) == A*x_yal(:,i)+B*u_yal(:,i) ];
end

% Terminal constraint
constraints = [constraints Oinf.A*x_yal(:,N+1) <= Oinf.b];

% Set options for YALMIP and solver
options = sdpsettings('verbose',0,'solver','quadprog','usex0',0,...
    'cachesolvers',1);

% Solve the problem
sol = optimize(constraints, objective, options);

% Retrieve solutions for plotting
xOpt = value(x_yal);
uOpt = value(u_yal);
JOpt = value(objective);

% Return feasibility
if sol.problem == 0
    feas = true;
else
    feas = false;
    fprintf('ERROR: Problem is infeasible!\n')
    return
%     xOpt = [];
%     uOpt = [];
%     JOpt = [];
end

end