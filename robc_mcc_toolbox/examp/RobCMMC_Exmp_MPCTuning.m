% Example 2: Robust MPC vs Classic MPC
% Author: Victor Daniel Reyes Dreke
clear,
% CT-SS Realization. Remark: Non-minimum phase system
Ac =  [0, 1; -6, -5];
Bc =  [0, 0.1; 1, 0];
Cc =  [-1, 1];

% DT-SS Realization
Ts =  0.1;                          % [s] Sampling time 
sysd = c2d(ss(Ac,Bc,Cc,[]),Ts);     % ZOH Discretization

% SS DT Matrices
A = sysd.A; B = sysd.B; C = sysd.C;

% Constraints definition
Xbound = [-1*ones(size(A,2),1), ones(size(A,2),1)]*1;  % [Xmin, Xmax] 
Ubound = [-1*ones(size(B,2),1), ones(size(B,2),1)]*10; % [umin, Umax]  

Pvec = [0, 0.4, 0, 0.4, zeros(1,4)];    % Pvec = [1 x nxm]

% Matching controllers design
% Case 1: LQR 
Q = eye(size(A,2)); R = 1e-2*eye(size(B,2));
[K_LQR,P_LQR] = dlqr(A,B,Q,R);

% Case 2: Constrained
[K_Const,P_Const]=getRobustK(A,B,[],Pvec,Xbound,Ubound,"constrained");

% MPC Tuning via Matching controller
SIGMA = 10^-2;      % Hyper-parameter
N = 5;              % Prediction Horizon
[Q,R,P,J,diagnostic]=matchingControllerDesign(K_Const,A,B,N,SIGMA);

%% Simulation 
Tsim = 1100;

Nk = 20;

x_ss = zeros(2,Tsim+1);
u_ss = zeros(2,Tsim);

Alpv =  @(p1,p2,p3,p4) A + [p1, p2; p3, p4]; 
Blpv =  @(p1,p2,p3,p4) B + [p1, p2; p3, p4]; 

dp =  zeros(length(Pvec), Nk-1);
for i=1:length(Pvec)
    if Pvec(i) == 0
        dp(i,:) = zeros(1,Nk-1);
    else
        dp(i,:) = -Pvec(i):(2.1*Pvec(i)/(Nk-1)):Pvec(i);% \delta \in ptemp
    end    
end

XUvec = cell(Nk,2);
% ---------------------------------------------------------------------- %
% %
x0 = [0.5; -0.5];
for i=1:Nk
    % Simulation of the Nk possibles uncertain \Delta A
    if i==1 % Nominal dynamics

        [x,u]= simMPCwRealPlant(A,B,A,B,Q,P,R,N,x0,...
                                        Xbound,Ubound,...
                                        x_ss,u_ss,Tsim);
    else % Uncertain dynamics
        j = i-1;
        try
        [x,u]= simMPCwRealPlant(A,B,...   
                        Alpv(dp(1,j),dp(2,j),dp(3,j),dp(4,j)),...
                        Blpv(dp(5,j),dp(6,j),dp(7,j),dp(8,j)),...
                        Q,P,R,N,x0,...
                        Xbound,Ubound,...
                        x_ss,u_ss,Tsim);
        catch  
            pause
        end
        
    end
    XUvec{i,1} = [x; zeros(2,size(x,2))];
    XUvec{i,2} = u;
end
% ---------------------------------------------------------------------- %
%%
tShow = 60; Tsim;
tIni = 1;
reference =  zeros(2,Tsim);

FigMainS=plotStateMPCv3(XUvec,Nk,Tsim,reference,tShow,tIni);
FigMainI=plotInputMPCv3(XUvec,Nk,Tsim,tShow,tIni);