% example_tennet_closed_loop_dynamics_aug
% Author: Victor Daniel Reyes
% ---------------------------------------------------------------------- %
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

% Augmented system
[A_on, B_on, C_on] = getAugmentedSystem(A_on, B_on, C_on);

nx = size(A_on,1);
nu = size(B_on,2);
np = size(C_on,1);

% Checking controllability and stability conditions.
checkSystem(A_on,B_on);


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
Xbound = [-1*ones(nx,1), 1*ones(nx,1)]*2;         % [Xmin, Xmax] 
Ubound = [-1*ones(nu,1), 1*ones(nu,1)]*0.5;         % [Umin, Umax]  

Pvec = [4e-2, 6e-2, 6e-2, 4e-2, zeros(1,4)];    % Pvec = [1 x nxm]

%%                     Controller Design sections                       %%
% Controller design
% Case 1: LQR 
Q = 1e2*eye(nx); R =1e-2*eye(nu);
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
x0 = zeros(nx,1);
xss = [0;0;0.67; -1e-4]; 

load('SavedParametersEx3_1_1.mat', 'prv');

% Open-loop Ok
Fig_poles_open = plotAugFullInformationOpen(MMC_ONSHORE,prv,Tsim,x0,xss);
% exportgraphics(gcf,'paper_aug_openLoop_i.pdf',...
%     'BackgroundColor','none','ContentType','vector')
% exportgraphics(gcf,'paper_aug_openLoop_i.emf',...
%     'BackgroundColor','none','ContentType','vector')

% Closed-loop
kn = 1e4;
Tsim =0.4*kn;
% x0 = [0.67; -1e-4];

x0 = zeros(nx,1);
xss = [zeros(2,Tsim+1); 
       zeros(1,0.05*kn), 0.5*ones(1,0.3*kn), 0.5*ones(1,0.05*kn+1); ...
       zeros(1,0.05*kn), 0.0*ones(1,0.3*kn), -0.1*ones(1,0.05*kn+1)]; 
nd = 1;
w = [zeros(nx,0.2*kn-nd), 0.04*ones(nx,nd), zeros(nx,0.2*kn+1)];

Fig_K_LQR=plotAugFullInformationClosed(MMC_ONSHORE,-K_LQR,prv,...
                                            Tsim,x0,xss,w);
% exportgraphics(gcf,'paper_aug_closedLoop_inf_KLQR.pdf',...
%     'BackgroundColor','none','ContentType','vector')
% exportgraphics(gcf,'paper_aug_closedLoop_inf_KLQR.emf',...
%     'BackgroundColor','none','ContentType','vector')

Fig_K_Robust=plotAugFullInformationClosed(MMC_ONSHORE,K_Robust,prv,...
                                            Tsim,x0,xss,w);
% exportgraphics(gcf,'paper_aug_closedLoop_inf_KRobust.pdf',...
%     'BackgroundColor','none','ContentType','vector')
% exportgraphics(gcf,'paper_aug_closedLoop_inf_KRobust.emf',...
%     'BackgroundColor','none','ContentType','vector')

Fig_K_Const=plotAugFullInformationClosed(MMC_ONSHORE,K_Const,prv,...
                                            Tsim,x0,xss,w);
% exportgraphics(gcf,'paper_aug_closedLoop_inf_KConst.pdf',...
%     'BackgroundColor','none','ContentType','vector')
% exportgraphics(gcf,'paper_aug_closedLoop_inf_KConst.emf',...
%     'BackgroundColor','none','ContentType','vector')

Fig_poles_closed=plotAugFullInformationClosed(MMC_ONSHORE,K_Class,prv,...
                                            Tsim,x0,xss,w);
%% 
% 
% Fig_DOA = plotDOAs(P_Const,P_LQR);
% 
% exportgraphics(gcf,'paper_aug_closedLoop_DOA.pdf',...
%     'BackgroundColor','none','ContentType','vector')
% exportgraphics(gcf,'paper_aug_closedLoop_DOA.emf',...
%     'BackgroundColor','none','ContentType','vector')

%%
% XHbound = [[-1; -1], [1; 1]];                % [Xmin, Xmax] 
% UHbound = [[-1; -1], [1; 1]];                % [Umin, Umax]  
% Fig_K_Const=plotFullInformationClosedMPC(MMC_ONSHORE,K_Const,...
%                                             Q_opt,R_opt,P_opt,N,XHbound,UHbound,prv,...
%                                             Tsim,x0,xss,w);
% exportgraphics(gcf,'paper_aug_closedLoop_inf_MPC.pdf',...
%     'BackgroundColor','none','ContentType','vector')
% exportgraphics(gcf,'paper_aug_closedLoop_inf_MPC.emf',...
%     'BackgroundColor','none','ContentType','vector')