function [K,P,Q,R,eval,Jval,diag]=getRobustKO(A,B,C,pvec,xbound,ubound,methodF)
% getRobustK(A,B,C,D,E,F,pvec,xbound,ubound) computes a robust state 
% feedback gain for an LPV representation of an uncertian linear model, 
% i.e., x(k+1) = A(p)x +B(p)u +E(p)w, such that \rho(A(p)+B(p)K)<1.
% Moreover, getRobustK(...,xbound,ubound) computes the weigthing matrix of 
% an MPC implementation using the Lyapunov inequality, as in  [1]. 
% ---------------------------------------------------------------------- %
% Inputs: 
% A     ->      [matrix function] state matrix
% B     ->      [matrix function] input matrix
% E     ->      [matrix function] exogenous input matrix
% C     ->      [matrix function] output matrix 
% D     ->      [matrix function] input feed matrix 
% F     ->      [matrix function] disturbance matrix 
% pvec  ->      [cell] {N x m+n} vertices of P scheduling variable space
% xbound ->     [matrix] {xmin, xmax} state box constraints
% ubound ->     [matrix] {umin, umax} input box constraints
% ---------------------------------------------------------------------- %
% Outputs:
% K     ->      [matrix] state feedback gain 
% P     ->      [matrix] terminal cost 
% Q     ->      [matrix] state weigthing matrix 
% R     ->      [matrix] input weigthing matrix
% eval  ->      [complex] closed-loop eignvalues
% Jval  ->      [scalar] value function
% diag  ->      [struct] diagnonstic report YALMIP
% ---------------------------------------------------------------------- %
% Authors: (i) Victor Daniel Reyes Dreke.
% ====================================================================== %
nx = size(A,1);
nu = size(B,2);
np = size(C,1);

neq = 2*length(pvec);
ABsys = cell(neq+1,2);

if isscalar(pvec)==true
    % Default uncertainty model,where only diagonal terms are uncertains

    vvec = kron([pvec, 0, 0, pvec]', [-1,1]); % Provisional 
    Au = [vvec;0*vvec];
    % Note: vvec = kron([pvec, 0, 0, pvec]', [-1,1]) - considers only an
    % uncertain variable. The [-1,1] needs to be replaced if we want to
    % include higher dimension (30-10-2025).
else    
    X = [0:2^(neq/2)-1];
    Y = int2bit(X,neq/2);
    vvec = 2*(Y - 0.5*ones(size(Y)));
    if size(pvec,2) > 1
        pvec = reshape(pvec,size(pvec,1)*size(pvec,2),1);
    end
    vvec = pvec.*vvec;
    [Au]=uniqueColumns(vvec);
end
% vvec = kron(pvec, [-1,1]);

% LPV Model, i.e., nominal model + uncertain model, 
% A(p) = A0 + \Delta A(p)  and B(p) = B0 + \Delta B(p) 

if nx==4
    % For augemented system 
    Alpv =  @(p1,p2,p3,p4) A + kron([1, 0; 1, 0],[p1, p2; p3, p4]); 
    Blpv =  @(p1,p2,p3,p4) B + kron([1; 1],[p1, p2; p3, p4]); 
elseif nx==2
    % For normal system 
    Alpv =  @(p1,p2,p3,p4) A + [p1, p2; p3, p4]; 
    Blpv =  @(p1,p2,p3,p4) B + [p1, p2; p3, p4]; 
end
for i=1:size(Au,2)+1
    if i==1
        ABsys{i,1} = A;
        ABsys{i,2} = B;
    else
        % Note: I have to find a way to generalize Alpv and Blpv
        ABsys{i,1} = Alpv(Au(1,i-1),Au(2,i-1),Au(3,i-1),Au(4,i-1));                      
        ABsys{i,2} = Blpv(Au(5,i-1),Au(6,i-1),Au(7,i-1),Au(8,i-1));
    end

end

% methodF = "robust";
% methodF = "constrained";
solverF = "MOSEK";

[P,K,~]=synthesisLpvK(ABsys,size(Au,2),...
                            methodF,solverF,...
                            xbound(:,2),xbound(:,1),...
                            ubound(:,2),ubound(:,1));

% Missing 
Q = [];
R = [];
eval= [];
Jval= [];
diag= [];
 
end