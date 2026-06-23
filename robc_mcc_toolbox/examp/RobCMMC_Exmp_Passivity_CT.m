% Example_Passivity_Conditions in CT
% Author: Victor Daniel Reyes
%

clear, clc, 
%% System Model
% Consider the following linear system describing a converter in dq
% framework, i.e., 
f0 = 50;                % [Hz] nominal frequency
w0 =  2*pi*f0;          % [rad/s] nominal angular frequency
L0 = 0.3;               % [H] nominal inductance
C0 = 0.2;               % [F] nominal capacitance
R0 = 0.8;               % [Ohm] nominal resistance


% State-space (SS) model
A = [-R0/L0, -w0; w0, -R0/L0];
B = [1/L0, 0; 0, 1/L0];
C = [1, 0; 0, 1];

% State-space (SS) dimensions
nx = size(B,1); nu = size(B,2); ny = size(C,1);
if nu~=ny
    warning('nu not equal to ny. Passivity can be checked only for system with the same number of ooutput thant inputs.')
end

% Ensuring no feedthrough term. 
D = zeros(ny,nu);

% State-space definition
sys_ss = ss(A,B,C,D);
sys_ss.InputName = {'u_d', 'u_q'};
sys_ss.OutputName = {'i_d', 'i_q'};
sys_ss.StateName = {'i_d', 'i_q'};


% Transfer function
% [NUM,DEN] = ss2tf(A,B,C,D,1);
sys_tf = tf(sys_ss);
sys_tf.InputName = {'u_d', 'u_q'};
sys_tf.OutputName = {'i_d', 'i_q'};
%%
% Checking Passivity

[P,diag] = checkPassivityCT(A,B,C,D);
[H_tf,idxPass,flagPass,win] = checkPassivityIndxCT(A,B,C,D,[]);

%%
%

% Response configuration
resOpt =  RespConfig; 
respOpt.Bias = [0,0];
respOpt.Amplitude = [0.5, 0.5];
respOpt.InitialState = [0.1,-0.1];
respOpt.Delay = 5;

tmax =  2.0;    % [s]
[yvec, tvec] =  step(sys_ss,tmax,resOpt);