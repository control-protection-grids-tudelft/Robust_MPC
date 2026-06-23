function [P,diagnostics] = checkPassivityCT(A,B,C,D)
% [P,diagnostics] = checkPassivityCT(A,B,C,D) examines the passivity of LTI
% systems based on the real positive lemma [1].
% -----------------------------------
% Inputs:   - (A,B,C,D) are the state-space matrices (standard notation)
%           - N is the prediction horizon.
% Outputs:  - P positive definitive matrix.
%           - diagnostics report of the optimization solver.
% ------------------------------------
% Author: Victor Daniel Reyes Dreke
% ------------------------------------
% References: 
% [1] Bao, J., & Lee, P. L. (2007). Process control: the passive 
% systems approach. London: Springer London.
% ------------------------------------

nx = size(B,1); 
nu = size(B,2); 
ny = size(C,1);

Pvar = sdpvar(nx,nx);
ops = sdpsettings('solver','mosek','verbose',0);
Constraints = [];

if norm(D, inf)==0
    % if D is zero, i.e., no feedthrough
    Constraints = [Constraints, A'*Pvar+Pvar*A<=-1e-6];
    Constraints = [Constraints, B'*Pvar == C];
else
    % General formulation
    Constraints = [Constraints, ...
                [A'*Pvar+Pvar*A, Pvar*B-C'; B'*Pvar-C, -D-D']<=-1e-8];
    Constraints = [Constraints, Pvar>=1e-6];
end

% Optimization
diagnostics = optimize(Constraints,[],ops);

P = value(Pvar);

end