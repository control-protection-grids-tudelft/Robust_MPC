% my_first_example is sample script presenting the entire workflow to use
% the robc_mmc_toolbox
% --------------------------------------------------------------------- %
% Author: Victor D. Reyes Dreke
% --------------------------------------------------------------------- %
% [1]   Dreke, Victor Daniel Reyes, Rahul Rane, and Aleksandra Lekić. 
% "Robust Tuning of Model Predictive Control for MMC-Based High-Voltage
% Power Systems." arXiv preprint arXiv:2606.11948 (2026).
% --------------------------------------------------------------------- %
clear all, clc,close all,
%
%%           Stage 1: Modeling stage
% -------------------------------------------------------------------
%  rated MMC papramteres based on CIGRE benchmark. 
Rr = 0.0060;        % [pu] AC transformer resistance
Lr = 0.0060;        % [pu] AC transformer inductance
Lm = 0.1800;        % [pu] Arm inductance
Rm = 0.1506;        % [pu] Arm resistancec
w = 1;              % [pu] angular velocity 
% ----------------------------------


% Building CT SS model of output current control (OCC)
[A, B, C] = getPlantModel(Rm, Rr, Lm, Lr, w, 'OCC');

% Discretization nominal  dynamics
% Note: Exact discretization for model with per-unit parameters must be
% done with per-unit smapling time. 
Ts = 5e-5;              % [s] sampling period 50ms
Tspu =  2*pi*50*Ts;     % [pu] per-unit samplig period considerinf 50Hz 
%                               nominal frequency.

sysnom = c2d(ss(A, B, C,[]),Tspu,'zoh'); % Nominal dynamics

% Dimension definition
nx = size(A,1);
nu = size(B,2);
np = size(C,1);
% ----------------------------------

% --------------------------------------------------------------------
% State and Input Soft Constraint definitions:
% ----------------------------------
isoft_max =  5; 
usoft_max =  0.01;

% Polytopic state and input constraints
Xbound = [-1*ones(nx,1), 1*ones(nx,1)]*isoft_max;            % [Xmin, Xmax] 
Ubound = [-1*ones(nu,1), 1*ones(nu,1)]*usoft_max;            % [Umin, Umax]  

% Vertices of X and U 
Vx = hyperboxVertices(Xbound(:,1), Xbound(:,2));
Vu = hyperboxVertices(Ubound(:,1), Ubound(:,2));
% ----------------------------------

% --------------------------------------------------------------------
% Uncertainty set P  definition
% ----------------------------------
Pvec = 1e-1*[4e-2, 6e-2, 6e-2, 4e-2, zeros(1,4)];   % Pvec = [1 x nxm]
% --------------------------------------------------------------------


%% Uncertainties defintion and scanning
% ----------------------------------
% Note: it is considered a range from 0.1% to 300% variation in the
% equivalent inductance and resistance values check [1, (3)].
% Note: This section is optinal can be commented.
% ----------------------------------

Req0 = Rr + Rm/2;
Leq0 = Lr + Lm/2;

cmin =0.1*[Req0;Leq0];
cmax =3.0*[Req0;Leq0];

[P1o, C1, C2] = fineGrid2D(cmin, cmax, 10);


CoefP = zeros(nx^2+nx*nu,length(P1o)+1);
for i=2:length(P1o)

    A = [-P1o(1,i)/P1o(2,i), -w; w, -P1o(1,i)/P1o(2,i)];
    B = [1/P1o(2,i), 0; 0, 1/P1o(2,i)];
    C = eye(2);

    % DT SS Representation
    sys1 = c2d(ss(A, B, C,[]),Tspu,'zoh');
    
    %
    CoefP(1,i) = sys1.A(1,1) - sysnom.A(1,1);
    CoefP(2,i) = sys1.A(1,2) - sysnom.A(1,2);
    CoefP(3,i) = sys1.A(2,1) - sysnom.A(2,1);
    CoefP(4,i) = sys1.A(2,2) - sysnom.A(2,2);
    

    CoefP(5,i) = sys1.B(1,1) - sysnom.B(1,1);
    CoefP(6,i) = sys1.B(1,2) - sysnom.B(1,2);
    CoefP(7,i) = sys1.B(2,1) - sysnom.B(2,1);
    CoefP(8,i) = sys1.B(2,2) - sysnom.B(2,2);

end

% ----------------------------
figure('WindowState','maximized')
tiledlayout(2,4);

for i=1:8
    Z = reshape(CoefP(i,2:end), size(C1));
    nexttile
    surf(C1, C2, Z)
    xlabel('$\Delta R_{\mathrm{eq}}$')
    ylabel('$\Delta L_{\mathrm{eq}}$')
    zlabel('$\rho$')
end

rho_max = max(CoefP(2,:),[],2)';
rho_min = min(CoefP(2,:),[],2)';

clear sys1
% End of the scanning uncertainty section
% --------------------------------------------------------------------

A = sysnom.A; B = sysnom.B; C = sysnom.C;  D = sysnom.D; 

%%           Stage 2: Control synthesis stage
% Note: This stage compose the design of robust state feedback controller
% and compute optimal wieight matrices based on the matching control
% algorithm.
% -------------------------------------------------------------------

% Checking controllability and stability conditions.
if checkSystem(A, B)
    disp('OCC system controllable');
else
    warning('OCC system did not pass the controllability check');
end

% ----------------------------------------------------------
% MPC parameters
% ----------------------------------
% Prediction Horizon
N = 4;
% ----------------------------------------------------------
% Controller Design
% ----------------------------------
% Robust state feeback gain
[K1o,P1o] = getRobustK(A,B,[],...
                        Pvec,Xbound,Ubound,"constrained");
% ----------------------------------------------------------
% Matching control MPC Tuning
% ----------------------------------
% Tolerance matching controller tuning
SIGMA = 10^2;

[Q,R,P,J,diagnostic] = matchingControlMPC(K1o,A,B,N,...
    0.5*Vx',SIGMA,[],[],'relax');
% ----------------------------------

%%      Stage 3: Online closed-loop simulation
% Note: 
% -------------------------------------------------------------------
% Experiment data:
kTT = 1e4;                              % Time magnitude
kT = kTT/10;                            % Time step magnitude
Tsim = 0.4*kTT;                         % Simulation time
t1 = 0.05e1; t2 = 0.3e1; t3 = 0.2e1;    % Step ratios


% Initial conditions: 
x0 =zeros(size(A,1),1);

% Reference design:
xss = [zeros(1,t1*kT), 0.5*ones(1,t2*kT), 0.5*ones(1,t1*kT+1); ...
       zeros(1,t1*kT), 0.0*ones(1,t2*kT), -0.1*ones(1,t1*kT+1)];


% Steady state reference
c = -(A-eye(size(A,1)))*xss;
uss = (B'*B)\(B'*c); % uss = pinv(B)*c;


% Hard constraints (operation limits)
i_max =  1.5; 
u_max =  1.5;

% Polytopic state and input hard constraints
Xbound = [-1*ones(nx,1), 1*ones(nx,1)]*i_max;            % [Xmin, Xmax] 
Ubound = [-1*ones(nu,1), 1*ones(nu,1)]*u_max;            % [Umin, Umax]  

% -------------------------------------------------------------------
% Closed-loop simulation
[xvec_m,uvec_m]= simMPCQuaprog(A,B,Q,P,R,N,x0,Xbound,Ubound,...
                                    xss,uss(1:2,:),Tsim);

[xvec_r,uvec_r]= simSystemSateFeedBack(A,B,K1o,x0,Tsim,xss,uss(1:2,:));

%% Plotting results

figure('WindowState','maximized')

plot(xvec_m')
hold on
plot(xss','--k')
title('RMPCC x')


figure('WindowState','maximized')
plot(xvec_r')
hold on
plot(xss','--k')
title('RCR x')

figure('WindowState','maximized')

plot(uvec_m')
hold on
plot(uss','--k')
title('RMPCC u')


figure('WindowState','maximized')
plot(uvec_r')
hold on
plot(uss','--k')
title('RCR u')
