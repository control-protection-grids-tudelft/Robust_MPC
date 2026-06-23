% example_tennet_closed_loop_dynamics_aug
% Script for Journal Publication
% Author: Victor Daniel Reyes
% ---------------------------------------------------------------------- %
%% 
clear, clc,
% ============== MMC Modeling
% Time parameters
Ts = 50e-6;     % [s] sampling time

% Load per-unit [p.u] MMC Parameters in a struct variable. 
MMC_ONSHORE  = getMMCAvgParameterPU('CIGRE_B457_DCS1_Onshore', Ts);  % [p.u]

% Building CT SS Representation: Onshore MMC
[AO_on, BO_on, CO_on,AC_on, BC_on, CC_on]=createMMCModel(MMC_ONSHORE);

nx = size(AO_on,1);
nu = size(BO_on,2);
np = size(CO_on,1);

% Checking controllability and stability conditions.
if checkSystem(AO_on,BO_on)
    disp('OCC system controllable');
else
    warning('OCC system did not pass the controllability check');
end

if checkSystem(AC_on,BC_on)
    disp('CCC system controllable');
else
    warning('CCC system did not pass the controllability check');
end
%% 
% ============== Controller Design Section

% ==============
% Design parameters:
% Polytopic state and input constraints
Xbound = [-1*ones(nx,1), 1*ones(nx,1)]*5;           % [Xmin, Xmax] 
Ubound = [-1*ones(nu,1), 1*ones(nu,1)]*0.01;         % [Umin, Umax]  

% % Provisional Polytopic state and input constraints
% Xbound = [-1*ones(nx,1), 1*ones(nx,1)]*2;         % [Xmin, Xmax] 
% Ubound = [-1*ones(nu,1), 1*ones(nu,1)]*0.5;         % [Umin, Umax] 

% Uncertainty set vertices (Assuming a symetric set centered in 0)
Pvec = 1e-1*[4e-2, 6e-2, 6e-2, 4e-2, zeros(1,4)];        % Pvec = [1 x nxm]
% Weighting matrices
Q = 0.5e0*eye(nx); R =2e1*eye(nu);
% Provisional
Q = 1e1*eye(nx); R =1e-2*eye(nu);
% Prediction Horizon
N = 5;
% Tolerance matching controller tuning
SIGMA = 10^-2;
% ==============

% ==============
% Output current control
% Case 1: LQR 
[KO_LQR, PO_LQR,~] = dlqr(AO_on,BO_on,Q,R);
% Case 2: Robust
tic
[KO_Robust,PO_Robust]=getRobustK(AO_on,BO_on,[],Pvec,Xbound,Ubound,"robust");
% Case 3: Constrained
toc
[KO_Const,PO_Const]=getRobustK(AO_on,BO_on,[],Pvec,Xbound,Ubound,"constrained");
% Case 4: Classic
[KO_Class,PO_Class]=getRobustK(AO_on,BO_on,[],Pvec,Xbound,Ubound,"classic");
% Case 5: MPC Tuning
[QO_opt,RO_opt,PO_opt,JO,Odiagnostic]=matchingControllerDesign(KO_Const,...
    AO_on,BO_on,N,SIGMA);
% ==============

% ==============
% Circulating current control
% Case 1: LQR 
[KC_LQR, PC_LQR,~] = dlqr(AC_on,BC_on,Q,R);
% Case 2: Robust
[KC_Robust,PC_Robust]=getRobustK(AC_on,BC_on,[],Pvec,Xbound,Ubound,"robust");
% Case 3: Constrained
[KC_Const,PC_Const]=getRobustK(AC_on,BC_on,[],Pvec,Xbound,Ubound,"constrained");
% Case 4: Classic
[KC_Class,PC_Class]=getRobustK(AC_on,BO_on,[],Pvec,Xbound,Ubound,"classic");
% Case 5: MPC Tuning
tic
[QC_opt,RC_opt,PC_opt,JC,Cdiagnostic]=matchingControllerDesign(KC_Const,...
    AC_on,BC_on,N,SIGMA);
toc
% ==============

%%
% Closed-loop stability analysis

% Eigen-values
% Uncertain realization parameters
Ustep = 0.1;
Umax = 3;
Umin =0.1;
NUstep = ceil(1+(Umax-Umin)/Ustep);

dRr = MMC_ONSHORE.Rr*(Umin:Ustep:Umax); 
dLr = MMC_ONSHORE.Lr*(Umin:Ustep:Umax);

dRac = MMC_ONSHORE.Rac*(Umin:Ustep:Umax);
dLac = MMC_ONSHORE.Lac*(Umin:Ustep:Umax);

AO_Ar = zeros(size(AO_on,1)*NUstep,size(AO_on,2));
BO_Ar = zeros(size(BO_on,1)*NUstep,size(BO_on,2));
CO_Ar = zeros(size(CO_on,1)*NUstep,size(CO_on,2));

AC_Ar = zeros(size(AC_on,1)*NUstep,size(AC_on,2));
BC_Ar = zeros(size(BC_on,1)*NUstep,size(BC_on,2));
CC_Ar = zeros(size(CC_on,1)*NUstep,size(CC_on,2));

% Matrix array
for k=1:NUstep

    type ='OCC';
    [AO_T,BO_T,CO_T]=createUncMMCModel(MMC_ONSHORE.Rr,dRac(k),MMC_ONSHORE.Lr,dLac(k),...
                                            MMC_ONSHORE.w,...
                                            MMC_ONSHORE.Ts_pu,...
                                            type);
    type ='CCC';
    [AC_T,BC_T,CC_T]=createUncMMCModel(MMC_ONSHORE.Rr,dRac(k),MMC_ONSHORE.Lr,dLac(k),...
                                            MMC_ONSHORE.w,...
                                            MMC_ONSHORE.Ts_pu,...
                                            type);

    AO_Ar(nx*(k-1)+1:nx*k,:) = AO_T;
    BO_Ar(nx*(k-1)+1:nx*k,:) = BO_T;
    CO_Ar(np*(k-1)+1:np*k,:) = CO_T;

    AC_Ar(nx*(k-1)+1:nx*k,:) = AC_T;
    BC_Ar(nx*(k-1)+1:nx*k,:) = BC_T;
    CC_Ar(np*(k-1)+1:np*k,:) = CC_T;
end

FigOC = plotUncertaintEigMMCv42(AO_on,BO_on,AO_Ar,BO_Ar,dRr,KO_Const,-KO_LQR);

FigOC = plotUncertaintEigMMCv2(AO_on,BO_on,AO_Ar,BO_Ar,dRr,KO_Const);
exportgraphics(FigOC,'EignUncertOC_RMPC_LQR_Total_v2.pdf',...
'BackgroundColor','none','ContentType','vector');

FigOCL = plotUncertaintEigMMC(AO_on,BO_on,AO_Ar,BO_Ar,dRr,-KO_LQR);

FigOCL = plotUncertaintEigMMCv2(AO_on,BO_on,AO_Ar,BO_Ar,dRr,-KO_LQR);
exportgraphics(FigOCL,'EignUncertOC_LQR_Total_v5.pdf',...
'BackgroundColor','none','ContentType','vector');

FigCC = plotUncertaintEigMMC(AC_on,BC_on,AC_Ar,BC_Ar,dRr,KC_Const);

Fig = plotUncertaintNyqistMMC(AO_on,BO_on,CO_on,...
    AO_Ar,BO_Ar,CO_Ar,dRr,KO_Const);
exportgraphics(Fig,'NyquistUncertOC_RMPC_Full_Total.pdf',...
'BackgroundColor','none','ContentType','vector');
Fig = plotUncertaintNyqistMMC(AO_on,BO_on,CO_on,...
    AO_Ar,BO_Ar,CO_Ar,dRr,-KO_LQR);
exportgraphics(Fig,'NyquistUncertOC_LQR_Full.pdf',...
'BackgroundColor','none','ContentType','vector');

exportgraphics(FigOC,'EignUncertOC_RMPC_Full.pdf',...
'BackgroundColor','none','ContentType','vector');
exportgraphics(FigCC,'EignUncertCC_RMPC.pdf',...
'BackgroundColor','none','ContentType','vector');

FigOC = plotUncertaintEigMMC(AO_on,BO_on,AO_Ar,BO_Ar,dRr,-KO_LQR);
% FigCC = plotUncertaintEigMMC(AC_on,BC_on,AC_Ar,BC_Ar,dRr,-KC_LQR);

exportgraphics(FigOC,'EignUncertOC_LQR_ZoomOut.pdf',...
'BackgroundColor','none','ContentType','vector');

%%
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



%%                     Controller Design sections                       %%


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

%%
% ==============
% Function section


function [AO_on,BO_on,CO_on,AC_on,BC_on,CC_on]=createMMCModel(MMC_ONSHORE)
% createMMCModel()
% 

% Building CT SS Representation: Onshore MMC
% Output Current Control
[A_OCc_on, B_OCc_on, C_OCc_on] = getPlantModel(MMC_ONSHORE.Rr, MMC_ONSHORE.Rac, ...
                            MMC_ONSHORE.Lr, MMC_ONSHORE.Lac, ...
                            MMC_ONSHORE.w, 'OCC');
% Circulating Current Control
[A_CCc_on, B_CCc_on, C_CCc_on] = getPlantModel(MMC_ONSHORE.Rr, MMC_ONSHORE.Rac, ...
                            MMC_ONSHORE.Lr, MMC_ONSHORE.Lac, ...
                            MMC_ONSHORE.w, 'CCC');
% DT SS Representation
% Output Current Control
SYS_OCOn_DISC = c2d(ss(A_OCc_on, B_OCc_on, C_OCc_on,[]),...
                            MMC_ONSHORE.Ts_pu,'zoh');
% Circulating Current Control
SYS_CCOn_DISC = c2d(ss(A_CCc_on, B_CCc_on, C_CCc_on,[]),...
                            MMC_ONSHORE.Ts_pu,'zoh');

% Rearrangement
% Output Current Control
AO_on = SYS_OCOn_DISC.A; BO_on = SYS_OCOn_DISC.B; CO_on = SYS_OCOn_DISC.C;
% Circulating Current Control
AC_on = SYS_CCOn_DISC.A; BC_on = SYS_CCOn_DISC.B; CC_on = SYS_CCOn_DISC.C;


% Augmented system construction
% Output Current Control
[AO_on, BO_on, CO_on] = getAugmentedSystem(AO_on, BO_on, CO_on);
% Circulating Current Control
[AC_on, BC_on, CC_on] = getAugmentedSystem(AC_on, BC_on, CC_on);

end

function [AO_on,BO_on,CO_on]=createUncMMCModel(dRr,dRac,dLr,dLac,w,Ts,type)
% createUncMMCModel()
% 

[A_OCc_on, B_OCc_on, C_OCc_on] = getPlantModel(dRr,dRac,dLr,dLac,...
                                                w,type);
SYS_OCOn_DISC = c2d(ss(A_OCc_on, B_OCc_on, C_OCc_on,[]),Ts,'zoh');

AO_on = SYS_OCOn_DISC.A; BO_on = SYS_OCOn_DISC.B; CO_on = SYS_OCOn_DISC.C;

% Output Current Control
[AO_on, BO_on, CO_on] = getAugmentedSystem(AO_on, BO_on, CO_on);


end