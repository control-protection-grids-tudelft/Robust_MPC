% example_tennet_closed_loop_dynamics
% Author: Victor Daniel Reyes
% ---------------------------------------------------------------------- %
clear, clc,
% Time parameters
Ts = 50e-6;     % [s] sampling time
% Load per-unit [p.u] MMC Parameters in a struct variable. 
MMC_ONSHORE  = getMMCAvgParameterPU('Tennet_2GW_Onshore', Ts);  % [p.u]
% Building CT SS Representation: Onshore MMC
[Ac_on, Bc_on, Cc_on] = getPlantModel(MMC_ONSHORE.Rr, MMC_ONSHORE.Rac, ...
                            MMC_ONSHORE.Lr, MMC_ONSHORE.Lac, ...
                            MMC_ONSHORE.w, 'OCC');
% DT SS Representation
SYS_On_DISC = c2d(ss(Ac_on, Bc_on, Cc_on,[]),MMC_ONSHORE.Ts_pu,'zoh');

% Rearrangement
A_on = SYS_On_DISC.A; 
B_on = SYS_On_DISC.B;   
C_on = SYS_On_DISC.C;

% Model uncertaint coefficient
NModel =  4;                    % Number of cases
% Edit this line to modify tolerance values ranges. Default: -0.5 to 0.5
prv_max =  [6e-1,...       % \Delta R_r,
            6e-1,...       % \Delta R_ac,
            4e-1,...       % \Delta L_r
            4e-1,...       % \Delta L_ac
            6e-1];         % \Delta w
prv = zeros(length(prv_max),NModel);  % Vector of uncertaint coefficient

for i=1:NModel
    for j=1:size(prv,1)
        prv(j,i)= (rand(1,1)-0.5)*prv_max(j);       %
    end
end

% Polytopic state and input constraints
Xbound = [[-1; -1], [1; 1]]*5;                  % [Xmin, Xmax] 
Ubound = [[-1; -1], [1; 1]]*0.2;                % [Umin, Umax]  

Pvec = [2e-2, 1.5e-1, 1.5e-1, 2e-2, zeros(1,4)];    % Pvec = [1 x nxm]

%%                     Controller Design sections                       %%
% Controller design
% Case 1: LQR 
Q = 1e4*eye(2); R =1e-2*eye(2);
[K_LQR, P_LQR,~] = dlqr(A_on,B_on,Q,R);
% Case 2: Robust
[K_Robust,P_Robust]=getRobustK(A_on,B_on,[],Pvec,Xbound,Ubound,"robust");
% Case 3: Constrained
[K_Const,P_Const]=getRobustK(A_on,B_on,[],Pvec,Xbound,Ubound,"constrained");
% Case 4: Classic
[K_Class,P_Class]=getRobustK(A_on,B_on,[],Pvec,Xbound,Ubound,"classic");

% MPC Tuning
SIGMA = 10^-2;
N = 5;
[Q_opt,R_opt,P_opt,J,diagnostic]=matchingControllerDesign(K_Const,...
    A_on,B_on,N,SIGMA);

%%                          Simulation sections                         %%
% Simulation time steps
Tsim =1e4;
% Initial conditions
x0 = [0; 0];
xss = [0.67; -1e-4]; 

% Load 
load('SavedParametersEx3_1_1.mat', 'prv');

% Open-loop
Fig_poles_open = plotFullInformationOpen(MMC_ONSHORE,prv,Tsim,x0,xss);

% exportgraphics(gcf,'paper_openLoop_i.pdf',...
%     'BackgroundColor','none','ContentType','vector')
% exportgraphics(gcf,'paper_openLoop_i.emf',...
%     'BackgroundColor','none','ContentType','vector')

% Closed-loop
Tsim =0.4e3;
x0 = [0; 0];
xss = [zeros(1,0.05e3), 0.5*ones(1,0.3e3), 0.5*ones(1,0.05e3+1); ...
       zeros(1,0.05e3), 0.0*ones(1,0.3e3), -0.1*ones(1,0.05e3+1)]; 
nd = 1;
w = [zeros(2,0.2e3-nd), 0.04*ones(2,nd), zeros(2,0.2e3+1)];

Fig_K_LQR=plotFullInformationClosed(MMC_ONSHORE,-K_LQR,prv,...
                                            Tsim,x0,xss,w);

% exportgraphics(gcf,'paper_closedLoop_inf_KLQR.pdf',...
%     'BackgroundColor','none','ContentType','vector')
% exportgraphics(gcf,'paper_closedLoop_inf_KLQR.emf',...
%     'BackgroundColor','none','ContentType','vector')

Fig_K_Robust=plotFullInformationClosed(MMC_ONSHORE,K_Robust,prv,...
                                            Tsim,x0,xss,w);

% exportgraphics(gcf,'paper_closedLoop_inf_KRobust.pdf',...
%     'BackgroundColor','none','ContentType','vector')
% exportgraphics(gcf,'paper_closedLoop_inf_KRobust.emf',...
%     'BackgroundColor','none','ContentType','vector')

Fig_K_Const=plotFullInformationClosed(MMC_ONSHORE,K_Const,prv,...
                                            Tsim,x0,xss,w);

% exportgraphics(gcf,'paper_closedLoop_inf_KConst.pdf',...
%     'BackgroundColor','none','ContentType','vector')
% exportgraphics(gcf,'paper_closedLoop_inf_KConst.emf',...
%     'BackgroundColor','none','ContentType','vector')

Fig_poles_closed=plotFullInformationClosed(MMC_ONSHORE,K_Class,prv,...
                                            Tsim,x0,xss,w);

%% 

Fig_DOA = plotDOAs(P_Const,P_LQR);

% exportgraphics(gcf,'paper_closedLoop_DOA.pdf',...
%     'BackgroundColor','none','ContentType','vector')
% exportgraphics(gcf,'paper_closedLoop_DOA.emf',...
%     'BackgroundColor','none','ContentType','vector')

%%
XHbound = [[-1; -1], [1; 1]];                % [Xmin, Xmax] 
UHbound = [[-1; -1], [1; 1]];                % [Umin, Umax]  
Fig_K_Const=plotFullInformationClosedMPC(MMC_ONSHORE,...
                                            Q_opt,R_opt,P_opt,N,XHbound,UHbound,prv,...
                                            Tsim,x0,xss,w);
% 
% exportgraphics(gcf,'paper_closedLoop_inf_MPC.pdf',...
%     'BackgroundColor','none','ContentType','vector')
% exportgraphics(gcf,'paper_closedLoop_inf_MPC.emf',...
%     'BackgroundColor','none','ContentType','vector')