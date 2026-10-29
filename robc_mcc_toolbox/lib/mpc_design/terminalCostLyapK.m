function [Pmax,K]=terminalCostLyapK(A,B,Q,R,K)
% terminalCostLyapunov(A,B,Q,R,K,P) implements the Lyapunov method to 
% calculate the termincal cost of the linear MPC to ensure stability and 
% recursive feasibility. In this case the bibligrophay are the lecture 4 
% and seminar 4 from Mircea Lazar MPC course. Yalimp is needed. 
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


% Y = sdpvar(m,n);

L1 = [O, (A*O+B*(K*O))', O, (K*O)';
    (A*O+B*(K*O)), O, zeros(size(O)), zeros(m,n)';
    O, zeros(size(O)), inv(Q), zeros(m,n)';
    (K*O), zeros(m,n), zeros(m,n), inv(R)]>=1e-6;


% L2 = O==inv(P);

% constraints = [L1; L2];
constraints = L1;
% diagnostics = optimize(constraints, -geomean(O));
diagnostics = optimize(constraints, []);


if diagnostics.problem == 0
    disp('Solver thinks it is feasible')
elseif diagnostics.problem == 1
    disp('Solver thinks it is infeasible')
else
    disp('Something else happened')
end

Pmax = value(O)^-1;

% K_lypv = value(Y)*value(O)^-1;
% trace(K_lypv'*K_lypv)

end