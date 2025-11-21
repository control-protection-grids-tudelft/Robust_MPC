%               PARAMTETER_LOAD_SFCK_DCS1
% This script is tailored to design a state feedback controller for output 
% current control (OCC) of MMC. The state feedback gain is robust against
% model uncertainties, i.e., robust controller. The controller synthesis
% relies on solving n LMI with MOSEK solver in combination with MPT3 
% toolbox. The MMMC's paramters are provided by the CIGRE model B 4.57.
% 
% Dir: 
% lib\ containts all MATLAB functions used
% References:
% [1]
% --------------------------------------------------------------------- %
clear, clc,
addpath lib\
% --------------------------------------------------------------------- %
% Simulations Settings
wbase = 2*pi*50;
Ts = 50*1e-6;              % [s^-1] Sampling period

% MMC Parameters
[MMCd] = parametersLibary('B4_57_DSC1_MMC5_SS');

% Constraints Definitions
CURRENT_BOUNDS =  [-MMCd.IbaseS_pk, MMCd.IbaseS_pk];
VOLTAGE_BOUNDS =  [-1.05, 1.05]*4;
CURRENTd_BOUNDS = [-MMCd.IbaseS_pk, MMCd.IbaseS_pk];
VOLTAGEd_BOUNDS = [-1.05, 1.05]/6;

% Parameters uncertanty
DELTA_IND_ARM = 0.15*MMCd.LacPU;    % [H] grid inductance variation


% Linear OCC per unit model. Check [Section 5.1 , 1]
[Ac, Bc, Cc] = getPlantModelPU(MMCd.Rm,MMCd.Rac,MMCd.Lm,MMCd.Lac,MMCd.w,...
                                MMCd.Ibase, MMCd.Vbase);

% Discrete-time linear OCC per unit model.
SYS_DISC = c2d(ss(Ac, Bc, Cc,[]),Ts,'zoh');

A = SYS_DISC.A; 
B = SYS_DISC.B;
C = SYS_DISC.C;

nx = size(A,1);
nu = size(B,2);
np = size(C,1);

% --------------------------------------------------------------------- %
% Controller synthesis

% Polytopic constraints the system [(), 1]
XBOUNDS = kron(ones(nx,1),CURRENTd_BOUNDS);
UBOUNDS = kron(ones(nu,1),VOLTAGEd_BOUNDS);

vectorMatrix = vectorMaking(A,XBOUNDS);

% rho vector (scheduling variables) defintion [(), 1]
pvec = [DELTA_IND_ARM,0,0,DELTA_IND_ARM,...
            Ts*(0.001/MMCd.Zbase),0,0,Ts*(0.001/MMCd.Zbase)];

% Controller synthesis function. Choose "robust" or "constrained".
% robust guarantees only quadratic stability.
% constrained gurantees quadratic stability and constraint satisfaction
[K,~,~,~,eval,Jval,diag]=getRobustK(A,B,[],[],[],[],pvec,...
                                    XBOUNDS,UBOUNDS,...
                                    "robust");

% Steady-state signal computation
Kuss = (B\(eye(2)-A));

% Saving parameters
save('LoadParameters_SFC2K.mat','Ts','A','B','C','K','Kuss')

