function [P,diagnostics] = checkPassivityLPVCT(A,B,C,D,pa,pb,pc,pd)
% [H_tf,idxPass,flagPass,win] = checkPassivityIndxCT(A,B,C,D,win) function
% to examine passivity of LPV systems based on the real positive lemma [1].
% -----------------------------------
% Inputs:   - (A,B,C,D) are the state-space matrices (standard notation)
%           - (pa,pb,pc,pd) are vectors of the rho vertices.
% Outputs:  - P positive definitive matrix.
%           - diagnostics report of the optimization solver.
% ------------------------------------
% Author: Victor Daniel Reyes Dreke
% ------------------------------------
% References: 
% [1] Bao, J., & Lee, P. L. (2007). Process control: the passive 
% systems approach. London: Springer London.
% ------------------------------------
% Initial value
A0 =  A(pa(1,:));
B0 =  B(pb(1,:));
C0 =  C(pc(1,:));
D0 =  D(pd(1,:));

% Passitive check based on the positive lemma
nx = size(B0,1); 
nu = size(B0,2); 
ny = size(C0,1);

Pvar = sdpvar(nx,nx);
vareps = sdpvar(1);
Objective = 0;
ops = sdpsettings('solver','mosek','verbose',0);
Constraints = [];

if norm(D0, inf)==0
    % if D is zero, i.e., no feedthrough
    for i=1:size(pa,1)
        % Constraints = [Constraints, A(pa(i,:))'*Pvar+Pvar*A(pa(i,:))<=-1e-10];
        Constraints = [Constraints, B(pb(i,:))'*Pvar == C(pc(i,:))];
        % Objective = Objective + norm(B(pb(i,:))'*Pvar-C(pc(i,:)), inf);
    end
    % Constraints = [Constraints, Pvar>=1e-10];
    
else
    % General formulation
    for i=1:size(pa,1)
        Constraints = [Constraints, ...
                [A(pa(i,:))'*Pvar+Pvar*A(pa(i,:)), ...
                Pvar*B(pb(i,:))-C(pc(i,:))'; ...
                B(pb(i,:))'*Pvar-C(pc(i,:)), ...
                -D(pd(i,:))-D(pd(i,:))']<=-1e-8];
    end
    Constraints = [Constraints, Pvar>=1e-6];
    % Objective = 0;
end

% Optimization
% diagnostics = optimize(Constraints,[],ops);
diagnostics = optimize(Constraints,Objective,ops);
P = value(Pvar);

end