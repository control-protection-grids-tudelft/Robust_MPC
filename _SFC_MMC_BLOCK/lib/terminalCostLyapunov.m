function [K_lypv,diagnostics] = terminalCostLyapunov(A,B,Q,R,P)
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


Y = sdpvar(m,n);

L1 = [O, (A*O+B*Y)', O, Y';
    (A*O+B*Y), O, zeros(size(O)), zeros(size(Y'));
    O, zeros(size(O)), inv(Q), zeros(size(Y'));
    Y, zeros(size(Y)), zeros(size(Y)), inv(R)]>=0;


L2 = O==inv(P);

constraints = [L1; L2];
diagnostics = optimize(constraints, trace(Y'*Y));

if diagnostics.problem == 0
    disp('Solver thinks it is feasible')
elseif diagnostics.problem == 1
    disp('Solver thinks it is infeasible')
else
    disp('Something else happened')
end


K_lypv = value(Y)*value(O)^-1;
 trace(K_lypv'*K_lypv)

end