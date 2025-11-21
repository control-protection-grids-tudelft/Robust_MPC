function [ Phi, Gamma ] = ABN2PhiGamma(A,B,N)
% ABN2PhiGamma(A,B,N) generates the prediction matrices Phi, Gamma
% corresponding to the prediction model of the linear MPC implementation as
% in [1].  The compact prediction model yields Xk = Phi x0 + Gamma Uk,
% where x0 = x(k), Xk = {x(0),..., x(N)} and Uk ={u(0),...,u(N)}.
% ---------------------------------------------------------------------- %
% Inputs:
% A     ->      [matrix] state matrix
% B     ->      [matrix] input matrix
% C     ->      [matrix] output matrix 
% N     ->      [scalar] prediction horizon
% ---------------------------------------------------------------------- %
% Outputs:
% Phi   ->      [matrix] state prediction matrix 
% Gamma ->      [matrix] input prediction matrix
% ---------------------------------------------------------------------- %
% Authors: (i) Duo Xu.
% ---------------------------------------------------------------------- %
% References:
% ---------------------------------------------------------------------- %
% [1]   Lazar, Mircea. "Model predictive control of hybrid systems: 
% Stability and robustness." (2006).
% ====================================================================== %
nx = size(B,1);
nu = size(B,2);

% Phi
Phi = A;
if N>1
    for j = 2:N
        Phi     = [Phi;A^j];
    end
end

% Gamma
Gamma = zeros(nx*N,nu*N);
for i = 1:N
    for j = 1:i
        Gamma((i-1)*nx+1:nx*i,(j-1)*nu+1:j*nu) = A^(i-j)*B;
    end
end
end

