function [A, B, C] = getPlantModel(Rarm, Rr, Larm, Lr, w, type)
% getPlantModel(type, Rarm, Rr, Larm, Lr, w) generates the A,B,C matrices
% corresponding to  continous-time state-space realization of the  MMC
% model,  i.e., x(k+1) = Ax(k) + Bu(k). 
% ---------------------------------------------------------------------- %
% Inputs: 
%  Rarm  ->     [Ohm] arm resistance
%  Rr    ->     [Ohm] grid resistance
%  Larm  ->     [Ohm] arm inductance
%  Lr    ->     [Ohm] grid inductance
%  type  ->     [string] defines the type of model to be used.
% ---------------------------------------------------------------------- %
% Outputs:
% A     ->      [matrix] state matrix
% B     ->      [matrix] input matrix
% C     ->      [matrix] output matrix 
% ---------------------------------------------------------------------- %
% Authors: (i) Msc. Rahul Rane, (ii) Victor Daniel Reyes Dreke.
% ====================================================================== %
switch nargin
    case 5 % OCC model
        warning('The default type model, i.e., OCC, is used.')
        % AC side equivalent parameters
        R_ac_eq = Rr + Rarm/2;
        L_ac_eq = Lr + Larm/2;
    
        % Continuous-time state-space model
        A = [-R_ac_eq/L_ac_eq, -w; w, -R_ac_eq/L_ac_eq];
        B = -[1/L_ac_eq, 0; 0, 1/L_ac_eq];
        C = eye(2);
    case 6
        type = string(type);
        if type == "occ"
            % AC side equivalent parameters
            R_ac_eq = Rr + Rarm/2;
            L_ac_eq = Lr + Larm/2;
        
            % Continuous-time state-space model
            A = [-R_ac_eq/L_ac_eq, -w; w, -R_ac_eq/L_ac_eq];
            B = -[1/L_ac_eq, 0; 0, 1/L_ac_eq];
            C = eye(2);
        elseif type == "ccc"
            % Continuous-time state-space model
            A = [-Rarm/Larm, 2*w; -2*w, -Rarm/Larm];
            B = -[-1/Larm, 0; 0, -1/Larm];
            C = eye(2);
        elseif type == "zcc"
            % Continuous-time state-space model
            A = -Rarm/Larm;
            B = -1/Larm;
            C = 1;
        else
            warning('The default type model, i.e., OCC, is used.')
            % AC side equivalent parameters
            R_ac_eq = Rr + Rarm/2;
            L_ac_eq = Lr + Larm/2;
        
            % Continuous-time state-space model
            A = [-R_ac_eq/L_ac_eq, -w; w, -R_ac_eq/L_ac_eq];
            B = -[1/L_ac_eq, 0; 0, 1/L_ac_eq];
            C = eye(2);
        end
    otherwise
        error('Check the input parameters for the function getPlantModel.')
end
% Provisiona line to match Rahul's model 16-11-2025.
B =-B;
end
