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

% Polytopic state and input constraints
Xbound = [[-1; -1], [1; 1]]*1;                  % [Xmin, Xmax] 
Ubound = [[-1; -1], [1; 1]]*0.2;                   % [Umin, Umax]  

% Model uncertaint coefficient
NModel =  100;                    % Number of cases
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

% method_d = "euler";
% [PAcVec,PBcVec,PCcVec,PAVec,PBVec,PCVec]=MMCValue2rho(MMC_ONSHORE,prv_max,method_d);

method_d = "zoh";
[PAcVec,PBcVec,PCcVec,PAVec,PBVec,PCVec]=MMCValue2rho(MMC_ONSHORE,prv_max,method_d);

% Uncertianty set building
nra = size(Ac_on,1)*size(Ac_on,2);
nrb = size(Bc_on,1)*size(Bc_on,2);

% Definition of the maximum values of rho vector. Check manual.
% Considering uncertainties in discrete time.
% Original
% Pvec = [2e-2, 1.5e-1, 1.5e-1, 2e-2, zeros(1,nrb)];    % Pvec = [1 x nxm]

Pvec = [reshape(PAVec',1,nra), 0*reshape(PBVec',1,nrb)];

% Au is set of all possible vertices of rho. Au is [nxm,p]. Check manual.
Au = rho2Pvec(Pvec);  % rho2Pvec compute the vertices of rho.

% Setting rho corresponding to A
PAvec = Au(1:nra,:);

% Setting rho corresponding to B
PBvec = Au(nra+1:nra+nrb,:);

%% RMC object construction
rmc_c = RMCClass(A_on,B_on,C_on,[],[],Xbound,Ubound);
% Defining the composite matrices
[ABsys,rmc_c] = rmc_c.getCompSystem(PAvec,PBvec);

%% Study Cases:
% Case 1: Classic Quadratic Controller
[K_Class,P_Class,~,~] = rmc_c.synthClassic();

% Case 2: Classic Robust Controller
[K_Robust,P_Robust,~,~] = rmc_c.synthClassicRobust();

% Case 3: Constrained Robust Controller
[K_Const,P_Const,diagnostics,~] = rmc_c.synthConstraintRobust();

% Case 4: using Ayari paper
x0 = 10*ones(size(Ac_on,1),1);

% Q = eye(size(Ac_on,1));
% R = eye(size(Bc_on,2));

Q = 1e4*eye(size(Ac_on,1)); R =1e-4*eye(size(Bc_on,2));
% Uncertain re-definition

pvec = [reshape(PAcVec',1,nra), 0*reshape(PBcVec',1,nrb)];

E1 = reshape(pvec(1:nra),size(Ac_on))';
E2 = reshape(pvec(nra+1:nra+nrb),size(Bc_on))';
F = eye(size(Ac_on,1));
D1 = eye(size(Ac_on,1),size(F,1));

[K_Opt,P_Opt,Pu,~,~] = rmc_c.synthOptRobust(Ac_on,Bc_on,E1,E2,D1,Q,R,x0);

% Case 5: LQR Controller
[K_LQR, P_LQR,~] = dlqr(A_on,B_on,Q,R);
K_LQR = -K_LQR;

%% Closed-loop
Tsim =0.4e4;
t1 = 0.05e1; t2 = 0.3e1; t3 = 0.2e1;
x0 = [0; 0];
xss = [zeros(1,t1*1e3), 0.5*ones(1,t2*1e3), 0.5*ones(1,t1*1e3+1); ...
       zeros(1,t1*1e3), 0.0*ones(1,t2*1e3), -0.1*ones(1,t1*1e3+1)]; 
nd = 1;
w = [zeros(2,t3*1e3-nd), 0.04*ones(2,nd), zeros(2,t3*1e3+1)];

Nk = 200;

PSimvec = ((rand(nra+nrb,Nk)-0.5)/0.52).*Pvec';
% [zeros(1,Nk); 
%         (rand(1,Nk)-0.5)*(0.02/0.5); 
%         zeros(1,Nk); 
%         (rand(1,Nk)-0.5)*(0.02/0.5); 
%         zeros(nrb,Nk)];
PSimvec = [zeros(size(Pvec,2),1),PSimvec];

% PSimvec = prv2PSimVec(prv,Ac_on,Bc_on,MMC_ONSHORE);

[Fig_fulL,XVecL,UVecL] = plotUncertaintSystem(A_on,B_on,K_LQR,PSimvec,x0,Tsim,Ts,w,xss,1.2);
% exportgraphics(gcf,'sest26_closedLoop_inf_K_LQR.pdf',...
%     'BackgroundColor','none','ContentType','vector')
% title('LQR')
[Fig_fulC,XVecC,UVecC] = plotUncertaintSystem(A_on,B_on,K_Const,PSimvec,x0,Tsim,Ts,w,xss,1.2);
% exportgraphics(gcf,'sest26_closedLoop_inf_K_Const.pdf',...
%     'BackgroundColor','none','ContentType','vector')
% title('Const')
[Fig_fulO,XVecO,UVecO] = plotUncertaintSystem(A_on,B_on,K_Opt,PSimvec,x0,Tsim,Ts,w,xss,1.2);
% exportgraphics(gcf,'sest26_closedLoop_inf_K_Opt.pdf',...
%     'BackgroundColor','none','ContentType','vector')
% title('Opt')