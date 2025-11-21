function [ Psi, Omega ] = QRPN2PsiOmega( Q,R,P,N )
%  QRPN2PsiOmega( Q,R,P,N ) generates the weighting matrices Psi, Omega
% corresponding to the quadratic cost function as in [1].The weighting 
% matrices Psi, Omega  yields Omega = diag(Q,...,Q,P) and 
% Psi = diag(R,...,R).
% ---------------------------------------------------------------------- %
% Inputs:
% Q     ->      [matrix] state weigth matrix
% R     ->      [matrix] input weigth matrix
% P     ->      [matrix] terminal cost matrix 
% N     ->      [scalar] prediction horizon
% ---------------------------------------------------------------------- %
% Outputs:
% Psi   ->      [matrix] compact input weighting matrix 
% Omega ->      [matrix] compact state weighting matrix
% ---------------------------------------------------------------------- %
% Authors: (i) Duo Xu and (ii) Victor Daniel Reyes Dreke
% ---------------------------------------------------------------------- %
% References:
% ---------------------------------------------------------------------- %
% [1]   Lazar, Mircea. "Model predictive control of hybrid systems: 
% Stability and robustness." (2006).
% ====================================================================== %
% Omega
if (N==1)
    Omega=P;
else
    Omega=Q;
    for j=2:(N-1)
        Omega=blkdiag(Omega,Q);
    end
    Omega=blkdiag(Omega,P);
%     omega=blkdiag(omega,Q);
end

% Psi
Psi = R;
if N>1
    for j = 2:N
        Psi = blkdiag(Psi,R);
    end
end
end

