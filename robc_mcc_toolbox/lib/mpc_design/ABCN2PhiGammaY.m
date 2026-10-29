function [PhiY, GammaY] = ABCN2PhiGammaY(A,B,C,N)
% [PhiY, GammaY] = ABCN2PhiGammaY(A,B,C,N) function to compute output-input 
% compact prediction model of LTI state-space models, such that 
% Y= PhiY*x(k)+ GammaY Uk, see [1]. 
% -----------------------------------
% Inputs:   - (A,B,C) are the state-space matrices (standard notation)
%           - N is the prediction horizon.
% Outputs:  - (PhiY, GammaY) are the prediction matrices.
% ------------------------------------
% Author: Victor Daniel Reyes Dreke
% ------------------------------------
% References: 
% [1] Wang, L. (2009). Model predictive control system design and 
% implementation using MATLAB®. London: Springer London.
% ------------------------------------

nx = size(B,1);
nu = size(B,2);
ny =  size(C,1);

% Phi
PhiY = C*A;
if N>1
    for j = 2:N
        PhiY     = [PhiY;C*A^j];
    end
end

% Gamma
GammaY = zeros(ny*N,nu*N);
for i = 1:N
    for j = 1:i
        GammaY((i-1)*ny+1:ny*i,(j-1)*nu+1:j*nu) = C*A^(i-j)*B;
    end
end
end
