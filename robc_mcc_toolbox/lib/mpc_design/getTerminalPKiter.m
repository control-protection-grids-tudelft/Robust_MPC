function [K,P,Ktr,Alpha,diagnostics] = getTerminalPKiter(A,B,Q,R,Kvec,Nvec,costFnc)
% getTerminalPLyap(A,B,Q,R) computes terminal cost (P) and gain (K) 
% leveraging Lyapunov inequalities criteria of the linear MPC to ensure 
% stability and recursive feasibility. In this case the bibligrophay are 
% the lecture 4 and seminar 4 from Mircea Lazar MPC course. 
% Note: Yalimp is needed. 
% ---------------------------------------------------------------------- %
% Inputs:
% ---------------------------------------------------------------------- %
% Outputs:
% ---------------------------------------------------------------------- %
% Authors: (i) Victor Daniel Reyes Dreke. 
% ---------------------------------------------------------------------- %
% References:
% ---------------------------------------------------------------------- %
% [1]   Dreke, Victor Daniel Reyes, and Mircea Lazar. "Practical deadbeat 
% mpc design via controller matching with applications in power 
% electronics." European Journal of Control 74 (2023): 100834.
% ====================================================================== %


n = size(A,1);
m = size(B,2);


O = sdpvar(n,n);
alph = sdpvar(Nvec,1);
c_delta = kron([10^-4; 10^4], ones(Nvec/2,1));
Y = zeros(m,n);


constraints = [];
cost = 0;

for j=1:Nvec

    switch costFnc
        case 'trace'
            K = Kvec((j-1)*m+1:j*m,:);
            cost = cost + trace((K*O)'*(K*O));
        otherwise
            cost = 0;
    end

    K = Kvec((j-1)*m+1:j*m,:);

    L1 = [O, (A*O+B*K*O)', O, (K*O)';
    (A*O+B*K*O), O, zeros(size(O)), zeros(size(Y'));
    O, zeros(size(O)), inv(Q), zeros(size(Y'));
    K*O, zeros(size(Y)), zeros(size(Y)), (alph(j,1))*inv(R)]>=0;

    constraints = [constraints; L1; alph(j,1)>=0];


end

options = sdpsettings('solver','mosek','verbose',0);

switch costFnc
    case 'trace'
        diagnostics = optimize(constraints, cost,options);
        % K = value(Y)*value(O)^-1;
    case 'geomean'
        % L2 = Y==K*O;
        % constraints = [constraints; L2];
        diagnostics = optimize(constraints, -geomean(O),options);
    otherwise
        % L2 = Y==K*O;
        % constraints = [constraints; L2];
        diagnostics = optimize(constraints, [],options);
end


if diagnostics.problem == 0
    disp('Solver thinks it is feasible')
elseif diagnostics.problem == 1
    disp('Solver thinks it is infeasible')
else
    disp('Something else happened')
end

P = value(O)^-1;
Ktr = trace(K'*K);
Alpha = []; %value(alph);
end