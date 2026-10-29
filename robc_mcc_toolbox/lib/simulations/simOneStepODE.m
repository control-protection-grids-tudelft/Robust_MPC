function [xkp, ykp, t_out, xk] = simOneStepODE(A,B,C,D,x0,u0, Ts)
% Simulates one sample step of a continuous-time state-space system
% using ode45 (from t = 0 to Ts). 
%
% Inputs:
%   A, B, C, D : State-space matrices
%   x0        : State at time k
%   u0        : Input held constant over interval
%   Ts         : Sampling time
%
% Outputs:
%   xkp     : State at time k+1
%   ykp     : Output at time k+1
%   t_out   : Time vector (internal integration)
%   xk      : State trajectory during integration

    % Define ODE function
    ode_fun = @(t, x) A*x + B*u0;

    % Integration interval
    t_span = [0 Ts];

    % Solve ODE
    [t_out, xk] = ode45(ode_fun, t_span, x0);

    % Take final state as next sample
    xkp = xk(end, :)';

    % Output at next step
    ykp = C*xkp + D*u0;
end