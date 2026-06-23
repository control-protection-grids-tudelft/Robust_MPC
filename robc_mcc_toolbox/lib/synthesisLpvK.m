function [P,Kx,gamma]=synthesisLpvK(ABsys,vertCount,methodF,solverF,xmax,xmin,umax,umin)
%  synthesisLpvK(ABsys,vertCount) implements an LMI synthesis of different
%  controller for an LPV system using Lyapunov inequality. There are
%  different methods: 
%  synthesisLpvK(ABsys,vertCount, 'method', 'classic') default
%  synthesisLpvK(ABsys,vertCount, 'method', 'constrained') 
%  synthesisLpvK(ABsys,vertCount, 'method', 'robust') 
%  implementation of the LMI synthesis of different controller using
%  Lyapunov inequality. 
% ---------------------------------------------------------------------- %
% Inputs: 
% ABsys ->      [cell] N x {A,B} cell array containing (A,B)
% xmin  ->      [nx x 1] column vector containing the minimum values of x
% xmax  ->      [nx x 1] column vector containing the maximum values of x
% umin  ->      [nu x 1] column vector containing the minimum values of u
% umax  ->      [nu x 1] column vector containing the maximum values of u
% methodF ->    {'classic','constrained','robust'} select the synthesis
%               method
% solverF ->    {'classic','constrained','robust'} select the synthesis
%               method
% ---------------------------------------------------------------------- %
% Outputs:
% K     ->      [matrix] state feedback gain 
% P     ->      [matrix] terminal cost 
% gamma ->      [scalar] gamma value
% ---------------------------------------------------------------------- %
% Authors: (i) Victor Daniel Reyes Dreke.
% ---------------------------------------------------------------------- %
% References:
% ---------------------------------------------------------------------- %
% [1] B. Kern and R. Findeisen, "Analysis and constrained control of 
% nonlinear interconnected systems exploiting positively invariant family 
% of sets," 52nd IEEE Conference on Decision and Control, Firenze, Italy,
% 2013, pp. 3806-3811, doi: 10.1109/CDC.2013.6760470.
% 
% [2] B. A. Francis, “The linear multivariable regulator problem,”
% SIAM Journal on Control and Optimization, vol. 15, DOI 10.1137/0315033, 
% no. 3, pp.486–505, 1977.
% 
% [3] B. Francis and W. Wonham, “The internal model principle of control
% theory,” Automatica, vol. 12, DOI https://doi.org/10.1016/0005-
% 1098(76)90006-6, no. 5, pp. 457–465, 1976.
% 
% [4] Gesser, Rodrigo S., Daniel M. Lima, and Julio E. Normey-Rico. 
% "Robust model predictive control: Implementation issues with comparative 
% analysis." IFAC-PapersOnLine 51.25 (2018): 478-483.
% 
% [5] IEEE OPEN JOURNAL OF THE INDUSTRIAL ELECTRONICS SOCIETY.
% VOLUME XX, 20XX 9
% =======================================================================
% Notes: 
% =======================================================================
%% 
if nargin == 2
    methodF = [];solverF = [];xmax= [];xmin = [];umax = [];umin= [];
elseif nargin == 3
    solverF = [];xmax = [];xmin = [];umax = [];umin = [];
elseif nargin == 4
    xmax = [];xmin = [];umax = [];umin = [];    
end
A  = ABsys{1,1};    % nominal A matrix
B  = ABsys{1,2};    % nominal B matrix

nx = size(B,1);     % dimensions of the state vector
nu = size(B,2);     % dimensions of the input vector

vj = vertCount;

% sdpvar varaibles definition, i.e., optimization variables. 
Z_i = sdpvar(nx,nx);            % inv(Z_i) =  P_i, due to Schur complement
Y_i = sdpvar(nu,nx,'full');     % Y_i defines K as K = Y_i*P_i
gamma_i = sdpvar(1,1);          % Scalar factor 

% ---------------------------------------------------------------------- %
%                   Classic Method 
% ---------------------------------------------------------------------- %
% Implementation (xxx) from [5], i.e., a conservative approach
% ---------------------------------------------------------------------- %
if methodF == "classic"
    vj = 1;
end

L1 = [];
tolVal = 1e-8; 

if (methodF=="robust" ||  methodF=="constrained")&& vj >= 2
    for i=2:vj+1
            M = [ Z_i               (ABsys{i,1}*Z_i+ABsys{i,2}*Y_i)';
                  (ABsys{i,1}*Z_i+ABsys{i,2}*Y_i)               Z_i];
                
            L1 = [L1, M >= tolVal];    
    end
end

M = [       Z_i                (A*Z_i+B*Y_i)';
     (A*Z_i+B*Y_i)                 Z_i];
 
L1 = [L1, M >= tolVal]; 
% ---------------------------------------------------------------------- %
%                   Constrained Method 
% ---------------------------------------------------------------------- %
% Implementation (xxx) from [5], i.e., a conservative approach
% For clasic box constraint situations, the following formulation is used 
% Implementation of Equation (43b) and (43c) from [5].
% ---------------------------------------------------------------------- %

if  methodF=="constrained"
    if nargin < 8
        XNOM = 1e4;
        warning('Constrained method chosen without defining constraints.')
        xmax = XNOM*ones(nx,1) ; xmin = -XNOM*ones(nx,1);
        umax = XNOM*ones(nx,1) ; umin = -XNOM*ones(nx,1);
        disp(['The states and inputs are constraind by XNOM =', num2str(XNOM)])
    elseif nargin == 8
    
        flagConst = false;
        XNOM = 1e4;    
        if isempty(xmax)==true
            xmax = XNOM*ones(nx,1);
            flagConst = true;
        elseif isempty(xmin)==true
            xmin = -XNOM*ones(nx,1);
            flagConst = true;
        elseif isempty(umax)==true
            umax = XNOM*ones(nx,1);
            flagConst = true;
        elseif isempty(umin)==true
            umin = -XNOM*ones(nx,1);
            flagConst = true;
        end
    
        if flagConst == true
            warning('Constrained method chosen without defining constraints.')
            disp(['The states and inputs are constraind by XNOM', num2str(XNOM)])
        end
    end
    
    nxConst = length(xmax);
    nuConst = length(umax);
        
    gij_max = diag(1./xmax);
    gij_min = diag(1./xmin);
        
    hij_max = diag(1./umax);
    hij_min = diag(1./umin);
    
    
    l3Temp = [];
    l4Temp = [];
    
    for k=1:nxConst
        tempMax = [Z_i, Z_i*gij_max(k,:)'; (Z_i*gij_max(k,:)')', gamma_i] >= 0;
        tempMin = [Z_i, Z_i*gij_min(k,:)'; (Z_i*gij_min(k,:)')', gamma_i] >= 0;
        l3Temp = [l3Temp, tempMax, tempMin];
    end
    
    for k=1:nuConst
        tempMax = [Z_i, (Y_i)'*hij_max(k,:)'; ((Y_i)'*hij_max(k,:)')', gamma_i] >= 0;
        tempMin = [Z_i, (Y_i)'*hij_min(k,:)'; ((Y_i)'*hij_min(k,:)')', gamma_i] >= 0;
        l4Temp = [l4Temp, tempMax, tempMin];
    end
    
    L3 = l3Temp;
    L4 = l4Temp;   
    
end

L8 = gamma_i == 1;
% ---------------------------------------------------------------------- %
%                   Solver 
% ---------------------------------------------------------------------- %
% Method selection
if methodF == "classic"
    constraints = [L1, L8]; % Here good
elseif methodF == "robust"
    constraints = [L1, L8]; % Here good
elseif methodF == "constrained"
    constraints = [L1, L3, L4, L8]; % Here good
else
    warning('Check variable methodF. Using default method,i.e., classic.')
    constraints = [L1, L8]; % Here good
end
% Solver selection
if solverF=="MOSEK"
    options = sdpsettings('solver','mosek','verbose',0);
    if  methodF=="constrained"
        diagnostics = optimize(constraints, -geomean(Z_i),options);
        % diagnostics = optimize(constraints, [],options); warning('Modified Line 188. Action needed it.')
    elseif  methodF=="robust"
        diagnostics = optimize(constraints, [],options);
    else
        diagnostics = optimize(constraints, [],options);
    end
    
    % diagnostics = optimize(constraints, -geomean(Z_i),options);
    % diagnostics = optimize(constraints, [],options);
elseif solverF=="SEDUMI"
    options = sdpsettings('solver','sedumi','sedumi.eps',1e-12);
    diagnostics = optimize(constraints, -geomean(Z_i),options);
elseif solverF=="SDPT3"
    diagnostics = optimize(constraints,-logdet(Z_i),...
                        sdpsettings('solver','sdpt3'));
else
    options = sdpsettings('solver','mosek','verbose',0);
    diagnostics = optimize(constraints, -geomean(Z_i),options);
end

if diagnostics.problem == 0
    disp('Solver thinks it is feasible')
elseif diagnostics.problem == 1
    disp('Solver thinks it is infeasible')
else
    disp('Something else happened')
    pause(2)
end

% ---------------------------------------------------------------------- %
%                   Solution
% ---------------------------------------------------------------------- %
P = value(Z_i)^(-1);
Kx = value(Y_i)*(value(Z_i)^(-1));
gamma = value(gamma_i);

end