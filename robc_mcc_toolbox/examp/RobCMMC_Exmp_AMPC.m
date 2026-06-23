% Example stability ingridients for the adaptive mpc [1]
% Author: Victor Daniel Reyes
% ---------------------------------------------------------------------- %
%%
close all, clear, clc,

%% ========================================================
%           Modeling section
% Comments: This section deals mainly with building a simple model of the
% OCC usin the generic state-space representation. I use personalized
% function to construct the model. Please use the robc_mmc_tool if you want
%  to use this models. If not you can always skip this and use you own
%  models.

% Time parameters
Ts = 50e-6;     % [s] sampling time

% Load per-unit [p.u] MMC Parameters in a struct variable. 
MMC_ONSHORE  = getMMCAvgParameterPU('CIGRE_B457_DCS1_Onshore', Ts);
% [p.u]

% Building CT SS Representation: Onshore MMC
[Ac_on, Bc_on, Cc_on] = getPlantModel(MMC_ONSHORE.Rr, MMC_ONSHORE.Rac, ...
                            MMC_ONSHORE.Lr, MMC_ONSHORE.Lac, ...
                            MMC_ONSHORE.w, 'OCC');

% DT SS Representation
SYS_On_DISC = c2d(ss(Ac_on, Bc_on, Cc_on,[]),MMC_ONSHORE.Ts_pu,'zoh');

% Rearrangement
A_on = SYS_On_DISC.A; 
B_on = SYS_On_DISC.B;   
C_on = SYS_On_DISC.C;

% AUgmented Model
% [A_on, B_on, C_on] = getAugmentedSystem(A_on, B_on, C_on);

nx = size(A_on,1);
nu = size(B_on,2);
np = size(C_on,1);

% Checking controllability and stability conditions.
checkSystem(A_on,B_on);

%% ========================================================
%           Terminal Ingridients design
% Comments: this is the actual core of the code. Here, I will implement
% Lemmas 4.2, i.e., the LMI. Note that most of the code is related to the
% construction of the element mention in the assumption of the Lemma. I
% mainly dedicate to compute the bounds of the matrix K based on the
% comutation of the K value for a certian combination  Q and R values.
% Again I use personalize functions so the use of the toolbox is highly
% recommended. 

% Initial Tuning Matrices
Q = eye(nx);
R = eye(nu);

% Window of values, i.e., ranges:
RngW =  3; % range of values

% Array containing set K,P,R values compute based on LQR design
Kvec = zeros(4,RngW);
Pvec = zeros(4,RngW);
Pmvec = zeros(4,RngW);
Rvec = zeros(1,RngW);

% Note: We are not restricted to LQR design we only need a set stabilizing
% K. 


costFnc = ''; % Hyperparameter to determine the type optimization 
% used to compute Lemma 4.2.
% Options:  ->'' default solution only verify the lmis
%           ->'trace' minimize the trace of K*O subject to the lmis
%           -> 'geomean' minimize the geometric mean of O subject to the lmis




% Ieration to compute a set of K depending on R = rscale*R with rscale 
% between 10^-4, 10^4.

for i=1:RngW
    rscale = 10^(i-RngW);
    Rvec(1,i) = rscale;
    
    % LQR 
    [K,P,~]= dlqr(A_on,B_on,Q,rscale*R);
    
    %
    [~,Pmax,~,~] = getTerminalPK(A_on,B_on,Q,rscale*R,-K,costFnc);
    Kvec(:,i) =[-K(1,:)'; -K(2,:)']; % Note: If using dlqr multiplication 
    % K times -1 is mandatory. 

    Pvec(:,i) =[P(1,:)'; P(2,:)'];
    % 
    Pmvec(:,i) =[Pmax(1,:)'; Pmax(2,:)']; % Checking code

end

rmax = max(Kvec(1:3,:),[],2);
rmin = min(Kvec(1:3,:),[],2);
Rho = genUncertaintVec(rmax,rmin);

nj = size(Rho,1);
Kj= zeros(size(Rho,1)*nu,nx);

for j=1:nj
    Kj((j-1)*nu+1:(j)*nu,:)= [Rho(j,1),Rho(j,2);Rho(j,3),Rho(j,1)];
end


[Kopt,Popt,Ktr,Alpha,diagnostics] = getTerminalPKiter(A_on,B_on,Q,R,Kj,nj,costFnc);

%%
%  Plot section:


cmap_ = orderedcolors('glow12');

sz =85; 
Fig = figure('WindowState','maximized');

% axis K values
ax1 = subplot(4,2,1);
ax2 = subplot(4,2,3);
ax3 = subplot(4,2,5);
ax4 = subplot(4,2,7);

% axis P values
ax5 = subplot(4,2,2);
ax6 = subplot(4,2,4);
ax7 = subplot(4,2,6);
ax8 = subplot(4,2,8);


% Plot the computed values of K gain. 
semilogx(ax1, Rvec',Kvec(1,:)', ...
    'LineWidth',2,'Color','#808080'), 
semilogx(ax2, Rvec',Kvec(2,:)', ...
    'LineWidth',2,'Color','#808080'), 
semilogx(ax3, Rvec',Kvec(3,:)', ...
    'LineWidth',2,'Color','#808080'), 
semilogx(ax4, Rvec',Kvec(4,:)', ...
    'LineWidth',2,'Color','#808080'), 

hold([ax1,ax2,ax3,ax4],"on") % scatter section

scatter(ax1, Rvec',Kvec(1,:)',...
    sz,cmap_(1:length(Kvec(1,:)),:),'filled','LineWidth',2, ...
    'MarkerEdgeColor','#808080'), 
scatter(ax2, Rvec',Kvec(2,:)',...
    sz,cmap_(1:length(Kvec(1,:)),:),'filled','LineWidth',2, ...
    'MarkerEdgeColor','#808080'), 
scatter(ax3, Rvec',Kvec(3,:)',...
    sz,cmap_(1:length(Kvec(1,:)),:),'filled','LineWidth',2, ...
    'MarkerEdgeColor','#808080'), 
scatter(ax4, Rvec',Kvec(4,:)',...
    sz,cmap_(1:length(Kvec(1,:)),:),'filled','LineWidth',2, ...
    'MarkerEdgeColor','#808080'), 



% Plot the computed values of K gain. 
semilogx(ax5, Rvec',[Pmvec(1,:)',Pmvec(1,:)'], ...
    'LineWidth',2,'Color','#808080'), 
semilogx(ax6, Rvec',[Pmvec(2,:)',Pmvec(2,:)'], ...
    'LineWidth',2,'Color','#808080'), 
semilogx(ax7, Rvec',[Pmvec(3,:)',Pmvec(3,:)'], ...
    'LineWidth',2,'Color','#808080'), 
semilogx(ax8, Rvec',[Pmvec(4,:)',Pmvec(4,:)'], ...
    'LineWidth',2,'Color','#808080'), 

hold([ax5,ax6,ax7,ax8],"on") % scatter section

scatter(ax5, Rvec',Pmvec(1,:)',...
    sz,cmap_(1:length(Pmvec(1,:)),:),'filled','LineWidth',2, ...
    'MarkerEdgeColor','#808080'), 
scatter(ax6, Rvec',Pmvec(2,:)',...
    sz,cmap_(1:length(Pmvec(1,:)),:),'filled','LineWidth',2, ...
    'MarkerEdgeColor','#808080'), 
scatter(ax7, Rvec',Pmvec(3,:)',...
    sz,cmap_(1:length(Pmvec(1,:)),:),'filled','LineWidth',2, ...
    'MarkerEdgeColor','#808080'), 
scatter(ax8, Rvec',Pmvec(4,:)',...
    sz,cmap_(1:length(Pmvec(1,:)),:),'filled','LineWidth',2, ...
    'MarkerEdgeColor','#808080'), 

% Optimal  P 

yline(ax5,Popt(1,1),'-','$P^{*}_{(1,1)}$',...
    'LineWidth',2.5, 'Color',[.80 0 .40],...
    'Interpreter','latex','FontSize',20)
yline(ax6,Popt(1,2),'-','$P^{*}_{(1,2)}$',...
    'LineWidth',2.5, 'Color',[.80 0 .40],...
    'Interpreter','latex','FontSize',20)
yline(ax7,Popt(2,1),'-','$P^{*}_{(2,1)}$',...
    'LineWidth',2.5, 'Color',[.80 0 .40],...
    'Interpreter','latex','FontSize',20)
yline(ax8,Popt(2,2),'-','$P^{*}_{(2,2)}$',...
    'LineWidth',2.5, 'Color',[.80 0 .40],...
    'Interpreter','latex','FontSize',20)

% Label section:
xlabel([ax4,ax8],'$\alpha(R=\alpha I_m)$')
title(ax1,'$K$-values')
title(ax5,'$P$-values')

legend(ax1,'$k_{(1,1)}$')
legend(ax2,'$k_{(1,2)}$')
legend(ax3,'$k_{(2,1)}$')
legend(ax4,'$k_{(2,2)}$')

legend(ax5,'$P_{(1,1)}$')
legend(ax6,'$P_{(1,2)}$')
legend(ax7,'$P_{(2,1)}$')
legend(ax8,'$P_{(2,2)}$')


% exportgraphics(Fig,'K_values.pdf',...
% 'BackgroundColor','none','ContentType','vector');

%%
% Checking stability criteria based on Lyapunov criteria

RngW2 = 100;
rscale = linspace(10^(1-RngW),10^(0),RngW2);

lambda_m = zeros(RngW2,2);
Q = eye(nx);
Rvec2 = zeros(RngW2,1);

for i=1:RngW2
    % rscale = 10^(i-RngW2);
    R = rscale(i)*eye(nu);
    Rvec2(i,1) = rscale(i);
    [Klqr,Plqr,~]= dlqr(A_on,B_on,Q,R);
    [~,Pmax,~,~] = getTerminalPK(A_on,B_on,Q,R,-Klqr,costFnc);
    % nidx = 10;
    % M = (A_on-B_on*Kopt)'*[Pmvec(1:2,nidx)';Pmvec(3:4,nidx)']*(A_on-B_on*Kopt)...
    %     -[Pmvec(1:2,nidx)';Pmvec(3:4,nidx)']+Q+Klqr'*R*Klqr; 
    M = (A_on-B_on*Klqr)'*Pmax*(A_on-B_on*Klqr)-Pmax+Q+Klqr'*R*Klqr; 
    M2 = (A_on-B_on*Klqr)'*Popt*(A_on-B_on*Klqr)-Popt+Q+Klqr'*R*Klqr; 
    
    lambda_m(i,1) = max(eig(M));
    lambda_m(i,2) = max(eig(M2));
  
end


cmap_ = prism(RngW2);

%%

Fig = ieee_fig();

newarray = [ .5020    .5020    .5020; ...
    .80 0 .40];
colororder(newarray)

semilogx(Rvec2,lambda_m,'-', ...
    'LineWidth',2); %,'Color','#808080');
hold 'on'
scatter(Rvec2,lambda_m(:,1),...
    sz,cmap_(1:RngW2,:),'filled','LineWidth',2, ...
    'MarkerEdgeColor','#808080');
scatter(Rvec2,lambda_m(:,2),...
    sz,cmap_(1:RngW2,:),'filled','LineWidth',2, ...
    'MarkerEdgeColor',[.80 0 .40]);

yr= yregion(0,Inf,FaceColor="r",EdgeColor=[0.4 0 0.7]);
yr.FaceAlpha = 0.1;
yr.EdgeAlpha = 0.5;
yr.LineWidth = 3;
yrs = yregion(-1e-6,-Inf,FaceColor="g",EdgeColor="#88FE45");
yrs.FaceAlpha = 0.1;
yrs.EdgeAlpha = 0.5;
yrs.LineWidth = 3;
ylabel('$\max(\lambda(M))$')
xlabel('$\alpha(R=\alpha I_m)$')
title('$ M:= (A+BK)^{\top}P(A+BK) - P +Q +K^{\top}RK$')

% ylim([-0.5 0.5])
% 

% exportgraphics(Fig,'Lambda_M_values.pdf',...
% 'BackgroundColor','none','ContentType','vector');