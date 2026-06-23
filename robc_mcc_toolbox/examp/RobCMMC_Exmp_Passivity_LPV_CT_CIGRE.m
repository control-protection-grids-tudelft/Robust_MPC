% Passivity Example
% Victor Daniel Reyes Dreke

clear, clc, 
%% System Model
% Consider the following linear system describing a converter in dq
% framework, i.e., (Rahul thesis)
f0 = 50;                    % [Hz] nominal frequency
w0 = 2*pi*f0;               % [rad/s] nominal angular frequency
La = 29e-3;                 % [H] nominal arm inductance
Lr = 35e-3;                 % [H] nominal grid inductance
C = 10e-3;                  % [F] nominal capacitance
Ra = 0*1e-2;                  % [Ohm] nominal arm resistance
Rr = 0.363;                 % [Ohm] nominal grid resistance
N = 200;
Ca = C/N;


type =  'Passivity';
[A,B,C,D]= getPlantModelLPV(type, Ra, Rr, La, Lr,Ca,f0);

%%
%

%
pa =[0.0*ones(1,8); ones(1,8)];
% pa =[0,0,0,0,0,0,0,0; 1,0,0,0,0,0,0,0];
pb = zeros(size(pa,1),1);
pc = zeros(size(pa,1),1);
pd = zeros(size(pa,1),1);

[P,diagnostics] = checkPassivityLPVCT(A,B,C,D,pa,pb,pc,pd)

% Provisional
P(isnan(P))=0; 

A(pa(2,:))
eig(A(pa(2,:)))



%% 
% System Response 
Tk =  2e4;
Ts = 1/50e3;
tsim = Ts*Tk; 

% State-space (SS) dimensions
nx = size(B(pb(1,:)),1); nu = size(B(pb(1,:)),2); ny = size(C(pc(1,:)),1);
if nu~=ny
    warning('nu not equal to ny. Passivity can be checked only for system with the same number of ooutput thant inputs.')
end
np = size(pa,2);
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
dvec = (0.5-rand(np,Tk+1))*0.2;
% dvec = [(0.5-rand(1,Tk+1))*0.2; zeros(7,Tk+1)];

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
