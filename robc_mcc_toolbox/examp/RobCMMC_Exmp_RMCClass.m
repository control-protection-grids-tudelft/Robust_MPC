% Example 3: Robust Controller Design with class
% Author: Victor Daniel Reyes Dreke

clear; clc;
% CT-SS Realization. Remark: Non-minimum phase system
Ac =  [0, 1; -6, -5];
Bc =  [0, 0.1; 1, 0];
% Bc =  [0.1;  1];
Cc =  [-1, 1];

% DT-SS Realization
Ts =  0.1;                          % [s] Sampling time 
sysd = c2d(ss(Ac,Bc,Cc,[]),Ts);     % ZOH Discretization

% SS DT Matrices
A = sysd.A; B = sysd.B; C = sysd.C;

% Polytopic state and input constraints
Xbound = [-[1; 1], [1; 1]];                % [Xmax, Xmin] 
Ubound = [-[1; 1], [1; 1]]*10;             % [Umax, Umin] 
% Ubound = [-1, 1]*10;                     % [Umax, Umin] 

% Uncertianty set building
nra = size(Ac,1)*size(Ac,2);
nrb = size(Bc,1)*size(Bc,2);
%pvec defines the maximum values of rho vector. Check manual. 
pvec = [0, 0.4, 0, 0.4, zeros(1,nrb)];  

% Au is set of all possible vertices of rho. Au is [nxm,p]. Check manual.
Au = rho2Pvec(pvec);  % rho2Pvec compute the vertices of rho.

% Setting rho corresponding to A
PAvec = Au(1:nra,:);

% Setting rho corresponding to B
PBvec = Au(nra+1:nra+nrb,:);

% Constructor of RMC object
rmc_c = RMCClass(A,B,C,[],[],Xbound,Ubound);
% Defining the composite matrices
[ABsys,rmc_c] = rmc_c.getCompSystem(PAvec,PBvec);

% Cases:
% Case 1: Classic Quadratic Controller
[K_Class,P_Class,~,~] = rmc_c.synthClassic();

% Case 2: Classic Robust Controller
[K_Robust,P_Robust,~,~] = rmc_c.synthClassicRobust();

% Case 3: Constrained Robust Controller
[K_Const,P_Const,~,~] = rmc_c.synthConstraintRobust();

% Case 4: using Ayari paper
% [K_Opt,P_Opt,Pu,~,~] = rmc_c.synthOptRobust(Ac,Bc,E1,E2,D1,Q,R,x0);


x0 = ones(size(Ac,1),1);

Q = eye(size(Ac,1));
R = eye(size(Bc,2));

% Uncertain re-definition
E1 = reshape(pvec(1:nra),size(Ac))';
E2 = reshape(pvec(nra+1:nra+nrb),size(Bc))';
F = eye(size(Ac,1));
D1 = eye(size(Ac,1),size(F,1));

[K_Opt,P_Opt,Pu,~,~] = rmc_c.synthOptRobust(Ac,Bc,E1,E2,D1,Q,R,x0);

%%
x0 = [0.5; -0.5];
Nk = 50;
Tsim =  60;
Pvec = [zeros(1,Nk); 
        (rand(1,Nk)-0.5)*(0.02/0.5); 
        zeros(1,Nk); 
        (rand(1,Nk)-0.5)*(0.02/0.5); 
        zeros(nrb,Nk)];
Pvec = [zeros(size(Pvec,1)),Pvec];

FigOpt = plotUncertaintSystem(A,B,K_Opt,Pvec,x0,Tsim,Ts,[],[],Ubound(1,2));
% FigOpt.WindowState ='maximized';
% exportgraphics(gcf,'paper_closedLoop_KOpt.pdf',...
%     'BackgroundColor','none','ContentType','vector')
% exportgraphics(gcf,'paper_closedLoop_KOpt.emf',...
%     'BackgroundColor','none','ContentType','vector')

FigConst = plotUncertaintSystem(A,B,K_Const,Pvec,x0,Tsim,Ts,[],[],Ubound(1,2));
% FigConst.WindowState ='maximized';
% exportgraphics(gcf,'paper_closedLoop_KConst.pdf',...
%     'BackgroundColor','none','ContentType','vector')
% exportgraphics(gcf,'paper_closedLoop_KConst.emf',...
%     'BackgroundColor','none','ContentType','vector')

%%
legendHead = {'Optimal','Constraint'};
Pcell = {P_Opt;P_Const};
Fig_DOA = plotDOAsv2(Pcell,legendHead);

% exportgraphics(gcf,'paper_plotDOAsv2.emf',...
%     'BackgroundColor','none','ContentType','vector')


% =======================
% CT-SS Realization. Remark: Non-minimum phase system
Ac2 =  [0, 1; -6, -5] + reshape(pvec(1:nra),size(Ac))';
Bc2 =  [0, 0.1; 1, 0] + reshape(pvec(nra+1:nra+nrb),size(Bc))';
% Bc =  [0.1;  1];
Cc =  [-1, 1];

% DT-SS Realization
Ts =  0.1;                            % [s] Sampling time 
sysd = c2d(ss(Ac2,Bc2,Cc,[]),Ts);     % ZOH Discretization

% SS DT Matrices
A2 = sysd.A; B2 = sysd.B; C2 = sysd.C;
% =======================


%% Simulation
Tsim =  1000;
x_ss = zeros(2,1); u_ss =zeros(2,1);
x0 = [0.5; -0.5];

xvec_cell = cell(4,1);
uvec_cell = cell(4,1);

[xvec,uvec] = simSystemSateFeedBack(A,B,K_Opt,x0,Tsim);
xvec_cell{1,1} = xvec;
uvec_cell{1,1} = uvec;
[xvec,uvec] = simSystemSateFeedBack(A,B,K_Const,x0,Tsim);
xvec_cell{2,1} = xvec;
uvec_cell{2,1} = uvec;
[xvec,uvec] = simSystemSateFeedBack(A,B,K_Class,x0,Tsim);
xvec_cell{3,1} = xvec;
uvec_cell{3,1} = uvec;
[xvec,uvec] = simSystemSateFeedBack(A,B,K_Robust,x0,Tsim);
xvec_cell{4,1} = xvec;
uvec_cell{4,1} = uvec;

%%
figure
for i=1:2
    for j=1:2
        ax = subplot(4,1,(i-1)*2+j);
        if i==1
            for k=1:4
                plot(ax,(0:Tsim)',xvec_cell{k,1}(j,:)','LineWidth',2.5)
                if k==1
                    hold(ax,"on")
                elseif k==4
                    legend(ax,'LQR','Classic','Robust','Constrained')
                    ylabel(ax,"$x_"+string(j)+"(k)$")
                    xlim(ax, [0 100])
                    ax.XMinorGrid = "on";
                    ax.YMinorGrid = "on";
                end
            end
        else
            for k=1:4
                plot(ax,(0:Tsim-1)',uvec_cell{k,1}(j,:)','LineWidth',2.5)
                if k==1
                    hold(ax,"on")
                elseif k==4
                    legend(ax,'LQR','Classic','Robust','Constrained')
                    ylabel(ax,"$u_"+string(j)+"(k)$")
                    xlim(ax, [0 100])
                    ax.XMinorGrid = "on";
                    ax.YMinorGrid = "on";
                end
            end
        end
    end
end