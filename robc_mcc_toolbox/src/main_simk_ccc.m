% Main simulation script CCC
% 
% 
%%  ======= System name
mmcModelHead = 'CIGRE_B457_DCS1_Onshore';

%%  ======= Time parameter definition
Ts = 50e-6;     % [s] sampling time
Tsim  = 10; TSim = 0.1;

%%  ======= Operating point definition
Id_ref = 0;
Iq_ref = 0;
x0 = [0.1,0.1]';

Vd_ = 0;
Vq_ = 0;

ulim = 1.2;

% Saving operating points
save('operating_point_c_parameters.mat', 'Id_ref','Iq_ref',...
                                       'Vd_','Vq_', 'ulim',...
                                       'Ts','TSim','mmcModelHead');
%%  ======= Model definition   

% Load per-unit [p.u] MMC Parameters in a struct variable. 
% Note: This function is within the examples folders
MMC_ONSHORE=getMMCAvgParameterPU(mmcModelHead, Ts);  % [p.u]

% Building CT SS Representation (Onshore MMC)
[Ac_on, Bc_on, Cc_on] = getPlantModel(MMC_ONSHORE.Rr, MMC_ONSHORE.Rac, ...
                            MMC_ONSHORE.Lr, MMC_ONSHORE.Lac, ...
                            MMC_ONSHORE.w, 'CCC');

% DT SS Representation
dsys = c2d(ss(Ac_on, Bc_on, Cc_on,[]),MMC_ONSHORE.Ts_pu,'zoh');

% DT SS Matrix
A = dsys.A; B =dsys.B; C = dsys.C; D = dsys.D; E =dsys.B;

% Saving ss matrices
save('ss_c_model_parameters.mat', 'A','B','C','D','E','Ts','mmcModelHead');

%%  ======= Feedforward design   

Kff = (B\(eye(size(A,1))-A));
% Saving ss matrices
save('ff_c_controller_parameters.mat', 'Kff','Ts');

%%  ======= State feedback design  

% Constraint definition
Xbound = [-[1; 1], [1; 1]]*5;            % [Xmax, Xmin] 
Ubound = [-[1; 1], [1; 1]]*0.1;            % [Umax, Umin] 

% Uncertainty realization
Pvec = 1e-1*[4e-2, 6e-2, 6e-2, 4e-2, zeros(1,4)];    % Pvec = [1 x nxm]

% Case 3: Constrained
[Kfb,P]=getRobustK(A,B,[],Pvec,Xbound,Ubound,"constrained");

% Saving state feedback matrices
save('fb_c_controller_parameters.mat','Kfb','P','Ts','mmcModelHead');

%%  ======= Augmented State feedback design  
% Augmented system
[A_on, B_on, C_on] = getAugmentedSystem(A, B, C);

% Dimensions
nx = size(A_on,1); nu = size(B_on,2); np = size(C_on,1);

% Check stability
checkSystem(A_on,B_on);

% Polytopic state and input constraints
Xbound = [-1*ones(nx,1), 1*ones(nx,1)]*5;           % [Xmin, Xmax] 
Ubound = [-1*ones(nu,1), 1*ones(nu,1)]*0.1;           % [Umin, Umax]  
% Pvec = [4e-2, 6e-2, 6e-2, 4e-2, zeros(1,4)];        % Pvec = [1 x nxm]

% Case 3: Constrained
[Kfbg,~]=getRobustK(A_on,B_on,[],Pvec,Xbound,Ubound,"constrained");

% Saving ss matrices
save('fbg_c_controller_parameters.mat', 'Kfbg','Ts','mmcModelHead');