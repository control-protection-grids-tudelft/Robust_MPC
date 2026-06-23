% Example_Passivity_Conditions in CT
% Author: Victor Daniel Reyes
%

clear, clc, 
%% System Model
% Consider the following linear system describing a converter in dq
% framework, i.e., 
f0 = 50;                    % [Hz] nominal frequency
w0 = 2*pi*f0;               % [rad/s] nominal angular frequency
L0 = 2.36e-3;                % [H] nominal inductance
C0 = 1.25e-3;               % [F] nominal capacitance
R0 = 50e-3;                 % [Ohm] nominal resistance


% State-space (SS) model
A = @(a) [a(1,1), a(1,2); a(1,3), a(1,4)]+[-R0/L0, -w0; w0, -R0/L0];
B = @(a) [a(1,1), a(1,2); a(1,3), a(1,4)]+[1/L0, 0; 0, 1/L0];
C = @(a) [a(1,1), a(1,2); a(1,3), a(1,4)]+[1, 0; 0, 1];

% State-space (SS) dimensions
nx = size(B([0,0,0,0]),1); nu = size(B([0,0,0,0]),2); ny = size(C([0,0,0,0]),1);
if nu~=ny
    warning('nu not equal to ny. Passivity can be checked only for system with the same number of ooutput thant inputs.')
end

% Ensuring no feedthrough term. 
D = @(a) [a(1,1), a(1,2); a(1,3), a(1,4)]+zeros(ny,nu);

%%
%
pa = [-20,-5,0,0; 20,5,0,0];
pb =[0,0,0,0; 0,0,0,0];
pc =[0,0,0,0; 0,0,0,0];
pd =[0,0,0,0; 0,0,0,0];


[P,diagnostics] = checkPassivityLPVCT(A,B,C,D,pa,pb,pc,pd)

%% 
% System Response 
Tk =  2e4;
Ts = 1/50e3;
tsim = Ts*Tk; 

% Intial conditions
x0 = rand(nx,1);
u0 = ones(nu,1)*0.1;

% Vectors Constructions
xvec = [x0,zeros(nx,Tk)];
yvec = [C(pc(1,:))*x0,zeros(ny,Tk)];
wvec = [u0, zeros(nu,Tk)];
wvec = [u0, [sin(2*pi*20*(Ts:Ts:tsim));cos(2*pi*20*(Ts:Ts:tsim))]];
svec = zeros(1,Tk+1);
Vvec = zeros(1,Tk+1);
dvec = [(0.5-rand(2,Tk+1))*5; zeros(2,Tk+1)];


for k=1:Tk

    u0 = wvec(:,k);

    [xkp, ykp, t_out, xk] = simOneStepODE(A(dvec(:,k)'),B(pb(1,:)),...
        C(pc(1,:)),D(pd(1,:)),x0,u0,Ts);

    xvec(:,k+1) = xkp;
    yvec(:,k+1) = ykp;
    % wvec(:,k+1) = u0;
    svec(1,k+1) = u0'*( C(pc(1,:))*x0);
    Vvec(1,k) = x0'*P*(A(dvec(:,k)')*x0+B(pb(1,:))*u0);
    x0 = xkp;
    % k
end

%%
% Plot
figure
ax = subplot(4,1,1);
plot(ax, (0:Ts:tsim)', dvec');
ylabel(ax,'$\delta(t)$')
xlabel(ax,'Time [s]')

ax = subplot(4,1,2);
plot(ax, (0:Ts:tsim)', wvec');

ax = subplot(4,1,3);
plot(ax, (0:Ts:tsim)', xvec');

ax = subplot(4,1,4);
plot(ax, (0:Ts:tsim)', svec');
hold(ax,'on');
plot(ax, (0:Ts:tsim)', Vvec');

ax = subplot(4,1,4);
plot(ax, (0:Ts:tsim)', Vvec'- svec');
