function [feas, xOpt, uOpt, JOpt] = solve_nmpc_quad(ts,Q,R,N,umin,umax,xmin,xmax,x0,E_obs)
% Solve the MPC problem for collision avoidance of a quadrotor

% Define system dimensions
NX = length(Q);
NU = length(R);

% Define decision variables
x_yal = sdpvar(NX,N+1);
u_yal = sdpvar(NU,N);

% Define the obstacle
[q_obs,Q_obs] = double(E_obs);

% assign(x_yal(:,1),x0);

% Define objective, constraints and terminal set
objective = 0;
for i = 1:N
    objective = objective + x_yal(:,i)'*Q*x_yal(:,i) + u_yal(:,i)'*R*u_yal(:,i);
end

constraints = x_yal(:,1)==x0;
for i = 1:N
    % state and input
    constraints = [constraints xmin <= x_yal(:,i) <= xmax...
                               umin <= u_yal(:,i) <= umax ];
    % dynamics
    constraints = [constraints x_yal(:,i+1) == RK4(x_yal(:,i), u_yal(:,i), ts, 5) ];
    
%     % obstacle avoidance
%     constraints = [constraints (x_yal(1:3,i)-q_obs)'*inv(Q_obs)*(x_yal(1:3,i)-q_obs)>=1];
end

% % obstacle avoidance (t=N+1)
% constraints = [constraints (x_yal(1:3,N+1)-q_obs)'*inv(Q_obs)*(x_yal(1:3,N+1)-q_obs)>=1];

% Terminal constraint
constraints = [constraints x_yal(:,N+1) == zeros(6,1)];

% Set options for YALMIP and solver
options = sdpsettings('verbose',0,'solver','snopt','usex0',0,'cachesolvers',1);
% Solve
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