function [xs,us,dhat]=estimateDisturbConst(x,x_prev,u_prev,Ld,Minv,dhat)
% estimateDisturbConst(x,x_prev,u_prev,Ld,Minv,dhat) disturbance estimator
% based on [1]. Needs to be use in combination with 
% getOffSetDisturbanceParam(.).
% ---------------------------------------------------------------------- %
% Inputs: 
% x          ->      [vector] current state at time k
% x_prev     ->      [vector] previous state at time k
% u_prev     ->      [vector] previous input at time k
% Ld         ->      [scalar] tuning parameter, such that, 0<Ld<=1.
% Minv       ->      [matrix] M output of getOffSetDisturbanceParam(.)
% dhat       ->      [vector] current value of predict distrubance 
% ---------------------------------------------------------------------- %
% Outputs:
% xs     ->      [vector] state steady state equilibrium 
% us     ->      [vector] input steady state equilibrium 
% dhat   ->      [vector] current value of predict distrubance
% ---------------------------------------------------------------------- %
% How to use its outcome in the closed-loop: 
% u = min(max(us - K*(x - xs), umin), umax);
% x_prev = x;  u_prev = u;
% ---------------------------------------------------------------------- %
% Authors: (i) Victor Daniel Reyes Dreke,
% ---------------------------------------------------------------------- %
% References:
% ---------------------------------------------------------------------- %
% [1]  Rawlings, J. B., Mayne, D. Q., & Diehl, M. (2017). Model Predictive
% Control: Theory, Computation, and Design, 2nd ed. Nob Hill Publishing.
% ====================================================================== %

res  = x - A*x_prev - B*u_prev;        % u_prev = input actually applied
dhat = dhat + Ld*(res - dhat);
s  = Minv*[-dhat; r];   
xs = s(1:n);   
us = s(n+1:end);


end