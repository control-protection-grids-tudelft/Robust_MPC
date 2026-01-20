% Example 2: Robust MPC vs Classic MPC with class
% Author: Victor Daniel Reyes Dreke

% CT-SS Realization. Remark: Non-minimum phase system
Ac =  [0, 1; -6, -5];
Bc =  [0, 0.1; 1, 0];
Cc =  [-1, 1];

% DT-SS Realization
Ts =  0.1;                          % [s] Sampling time 
sysd = c2d(ss(Ac,Bc,Cc,[]),Ts);     % ZOH Discretization

% SS DT Matrices
A = sysd.A; B = sysd.B; C = sysd.C;


% Polytopic state and input constraints
Xbound = [-[1; 1], [1; 1]];            % [Xmin, Xmax] 
Ubound = [-[1; 1], [1; 1]]*10;         % [Umin, Umax]  
Pvec = [0, 0.4, 0, 0.4, zeros(1,4)];   % Pvec = [1 x nxm]

% MPC Hyper-parameters
N =  105;                                 % [N] prediction horizon
Q = eye(2); R = 1e-2*eye(2);            % Weigthing matrices

% Build object MPC
mpc_constrained = MPCClass(A,B,C,Pvec,Xbound,Ubound,N,Q,R); % Robust MPC
mpc_classic = MPCClass(A,B,C,Pvec,Xbound,Ubound,N,Q,R);     % Classic MPC
% Input_rate based MPC
mpc_input_rate = MPCClass(A,B,C,Pvec,Xbound,Ubound,N,Q,R,'input_rate');     

% Create OCP formulation
sigma = 10^-2;              % Hyper-parameters

% Tuning Methods => "robust", "constrained", "classic"
mpc_constrained = mpc_constrained.createOCP(sigma,"constrained",'sti','Hildreth');
mpc_classic = mpc_classic.createOCP(sigma,'dlqr','sti','Hildreth');
mpc_input_rate = mpc_input_rate.createOCP(sigma,'dlqr','sti','Hildreth');

x0 = [0.5; -0.5];
u0 = zeros(2,1);
% Solve QP problem
u_qp_const = mpc_constrained.solveOneQP(x0,u0);
u_qp_clas = mpc_classic.solveOneQP(x0,u0);
u_qp_inpt = mpc_input_rate.solveOneQP(x0,u0);
