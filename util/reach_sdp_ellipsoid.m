function output = reach_sdp_ellipsoid(net, input_set, repeated, verbose, sys)
% Works for ReLU activation, input and output sets being both ellipsoids, 
% and input projector.

%% Initialization and calculating preactivation
% Input set.
E_in = input_set.set;
input_type = input_set.type;

% Box outer-approximation of the input ellipsoid for interval arithmatic.
if strcmp(input_type,'ellipsoid') 
    [q_in,Q_in] = double(E_in);
    x_max = q_in + sqrt(diag(Q_in));
    x_min = q_in - sqrt(diag(Q_in));
else
    error('Unsupported input set!')
end

% Number of hidden layers (activation layers).
num_layers = length(net.dims)-2;

% State dimension (input to the DNN - the current state).
dim_x = net.dims(1);

% Output dimension (output of the DNN - the controls).
dim_u = net.dims(end);

% Number of neurons in the last hidden layer.
dim_last_hidden = dim_u;

% Total number of neurons.
num_neurons = sum(net.dims(2:end-1)) + 2*dim_u;

% Dimension of the SDP problem / M matrix.
dim_sdp = dim_x + num_neurons + 1;

% Preactivation calculation with interval arithmatic.
[Y_min,Y_max,X_min,X_max] = interval_arithmatic(net,x_min,x_max);

X_min = [X_min; zeros(2*dim_u,1)];
X_max = [X_max; zeros(2*dim_u,1)];

Wout = net.weights{end};
bout = net.biases{end}(:);

Wout = double(Wout);
bout = double(bout);

I_pos = intersect(find(Y_min>0),find(Y_max>0));
I_neg = intersect(find(Y_min<0),find(Y_max<0));

if(~exist('net.A'))
    A = [];
    aa = [];
    for i=1:num_layers % 0 to l-1
        A = blkdiag(A, blkdiag(net.weights{i}));
        aa = [aa; net.biases{i}(:)];
    end
    A  = blkdiag(A, Wout, -eye(dim_u));
    A  = [A zeros(size(A,1),dim_u)];
    aa = [aa; bout-sys.ulb; sys.uub];
    B  = [zeros(num_neurons,dim_x), blkdiag(eye(num_neurons-dim_u),...
        -eye(dim_u))];
    bb = [zeros(num_neurons-2*dim_u,1); -sys.ulb; sys.uub];
end

A = double(A);
B = double(B);
aa = double(aa);
bb = double(bb);

% ReLU slope-restricted nonlinearity.
alpha = 0;
beta = 1;

% System matrices.
As = sys.A;
Bs = sys.B;
try
  Es = sys.E;
  es = sys.e;
catch
  Es = zeros(dim_x,1);
  es = 0;
end

% Setting up CVX.
if(verbose)
    cvx_begin sdp
else
    cvx_begin sdp quiet
end

cvx_solver mosek


%% Input quadratic
variable mu_in nonnegative
[A_in,b_in] = ell_to_Ab(E_in);
P = mu_in*[-A_in'*A_in -A_in'*b_in; -b_in'*A_in 1-b_in'*b_in];

% Lifting matrix LP.
LP = zeros(dim_x+1,dim_sdp);
LP(1:dim_x,1:dim_x) = eye(dim_x);
LP(end,end) = 1;

Min = LP'*P*LP;


%% Output quadratic
variable A_out(dim_x,dim_x) symmetric
variable b_out(dim_x,1)

% Matrices for the Schur complement.
Lout = [As zeros(dim_x,dim_sdp-dim_x-dim_last_hidden-1) Bs Es*es];
Lout = [Lout; zeros(1,size(Lout,2)-1) 1];
F = Lout.'*[A_out.'; b_out.'];
e = [zeros(dim_sdp-1,1); 1];


%% QC for activation functions
T = zeros(num_neurons);

% Repeated nonlinearity.
if(repeated)
    II = eye(num_neurons);
    C = [];
    if(numel(I_pos)>1)
        C = nchoosek(I_pos,2);
    end
    if(numel(I_neg)>1)
        C = [C;nchoosek(I_neg,2)];
    end
    m = size(C,1);
    if(m>0)
        variable zeta(m,1)
        E = II(:,C(:,1))-II(:,C(:,2));
        T = E*diag(zeta)*E';
    end
end
variable nu(num_neurons,1) nonnegative
variable lambda(num_neurons,1)
variable eta(num_neurons,1) nonnegative
variable D_diag(num_neurons,1) nonnegative

Dt = diag(D_diag);
D = blkdiag(eye(num_neurons-2*dim_u),...
    0*eye(2*dim_u))*Dt*blkdiag(eye(num_neurons-2*dim_u), 0*eye(2*dim_u));

Q11 = -2*alpha*beta*(diag(lambda)+T);
Q12 = (alpha+beta)*(diag(lambda)+T);
Q13 = -nu;
Q21 = Q12.';
Q22 = -2*(diag(lambda))-2*T-(D+D.');
Q23 = eta+nu+D*X_min+D.'*X_max;
Q31 = Q13.';
Q32 = Q23.';
Q33 = 0-X_min'*D.'*X_max-X_max'*D*X_min;
    
Q_out = [Q11 Q12 Q13; Q21 Q22 Q23; Q31 Q32 Q33];

% Lifting matrix for Q.
LQ = [A aa; B bb; zeros(1,size(B,2)) 1];
Mmid = LQ'*Q_out*LQ;

clear A B aa bb;
clear Q11 Q12 Q13 Q21 Q22 Q23 Q31 Q32 Q33;


%% Formulate and solve the SDP
% Schur complement for outer approximation.
M = [ Min+Mmid-e*e' F; F.' -eye(dim_x)];

% Solve the SDP.
maximize log_det(A_out)

subject to

M <= 0;

cvx_end

message = ['Solver time: ', num2str(cvx_cputime)];
disp(message);

q_out = -A_out^-1*b_out;
Q_out = (A_out*A_out)^-1*mu_in^2;

output.ell = ellipsoid(q_out,(Q_out+Q_out')/2);
output.cpu_time = cvx_cputime;
output.solver_status = cvx_status;
end

