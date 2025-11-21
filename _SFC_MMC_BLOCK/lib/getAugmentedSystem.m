function [Ag, Bg, Cg] = getAugmentedSystem(A, B, C)
% getAugmentedSystem(A, B, C) generates the augmented state-space
% realization correspoding to discrete-time state-space models. The
% augemented system is described as in (4) from [1].
% ---------------------------------------------------------------------- %
% Inputs:
% A     ->      [matrix] state matrix
% B     ->      [matrix] input matrix
% C     ->      [matrix] output matrix 
% ---------------------------------------------------------------------- %
% Outputs:
% Ag    ->      [matrix] state matrix
% Bg    ->      [matrix] input matrix
% Cg    ->      [matrix] output matrix 
% ---------------------------------------------------------------------- %
% Authors: (i) Msc. Rahul Rane, (ii) Victor Daniel Reyes Dreke.
% -------------------------------------------------------------------- %
% References:
% -------------------------------------------------------------------- %
% [1]   Shetgaonkar, Ajay, et al. "Model predictive control and protection 
% of MMC-based MTDC power systems." Int. J. Electr. Power Energy Syst. 146
% (2023): 108710.
% =================================================================== %
    nx = size(A,1); 
    ny = size(C,1);

    Ag = blkdiag(A, eye(ny));
    Ag(nx+1:end,1:nx) = C * A;
    Bg = [B; C * B];
    Cg = [zeros(nx), eye(ny)];
end
