function [PhiY, GammaY] = ABCN2PhiGammaY(A,B,C,N)
%ABN2PhiGamma Summary of this function goes here
%   Detailed explanation goes here

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
