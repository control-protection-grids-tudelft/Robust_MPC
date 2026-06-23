function [A,B,C,D]= getPlantModelLPV(type, Ra, Rr, La, Lr,Ca,f0)
% getPlantModelLPV(type, Ra, Rr, La, Lr,Ca,w) generates the A,B,C,D 
% function matrices corresponding to  continous-time LPV state-space 
% realization of the  MMC model. Note: Depending the application the model
% structure.
% ---------------------------------------------------------------------- %
% Inputs: 
%  Ra  ->     [Ohm]     arm resistance
%  Rr  ->     [Ohm]     grid resistance
%  La  ->     [H]       arm inductance
%  Lr  ->     [H]       grid inductance
%  Ca  ->     [F]       arm capacitance
%  f0  ->     [Hz]      nominal frequency 
%  type  ->     [string] defines the type of model to be used.
% ---------------------------------------------------------------------- %
% Outputs:
% A     ->      [matrix] function matrix
% B     ->      [matrix] function matrix
% C     ->      [matrix] function matrix 
% ---------------------------------------------------------------------- %
% Authors: (i) Victor Daniel Reyes Dreke.
% ====================================================================== %
if ischar(type)==false
    warning('Type needs to be char type');
end

switch type
    case 'PPassivity'
        % Passivity analysis single phase MMC
        Leq = La+2*Lr; 
        Req = Ra+2*Rr;
        A = @(a)[-Req/Leq,0,0,0; 0,-Ra/La,0,0; 0,0,0,0; 0,0,0,0]+...
            [0,0,a(1,1)*(-1/Leq),a(1,2)*(1/Leq);
             0,0,a(1,3)*(-1/(4*La)),a(1,4)*(-1/(4*La));
             a(1,5)*(1/(2*Ca)), a(1,6)*(1/(Ca)),0,0;
             a(1,7)*(1/(2*Ca)), a(1,8)*(-1/(Ca)),0,0;];
        % Important: to check number of inputs must be equal to number 
        % of outputs.
        B = @(a)[-2/Leq 0; 0, 1/(4*La); zeros(2,2)]+zeros(4,2)*a; 
        C = @(a)[1 zeros(1,3); 0, 1, zeros(1,2)]+zeros(2,4)*a; 
        D = @(a)zeros(2,2)+zeros(2,2)*a;
    case 'Passivity'
        Leq = Lr+La/2; 
        Req = Rr+Ra/2;
        A = @(a)[-Req/Leq,0,0,0; 0,-Ra/La,0,0; 0,0,0,0; 0,0,0,0]+...
            [0,0,a(1,1)*(-1/(2*Leq)),a(1,2)*(-1/(2*Leq));
             0,0,a(1,3)*(-1/(2*La)),a(1,4)*(-1/(2*La));
             a(1,5)*(1/(4*Ca)), a(1,6)*(1/(2*Ca)),0,0;
             a(1,7)*(1/(4*Ca)), a(1,8)*(1/(2*Ca)),0,0;];
        % Important: to check number of inputs must be equal to number 
        % of outputs.
        B = @(a)[1/Leq 0; 0, 1/(2*La); zeros(2,2)]+zeros(4,2)*a; %Cambie el signo del voltage de la red
        C = @(a)[1 zeros(1,3); 0, 1, zeros(1,2)]+zeros(2,4)*a; 
        D = @(a)zeros(2,2)+zeros(2,2)*a;
    otherwise
        warning('None selected type of model')
end

end