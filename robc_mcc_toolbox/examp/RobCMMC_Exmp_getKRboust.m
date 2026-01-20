% Example 1: Computation of different controllers using getKRobust()
% Author: Victor Daniel Reyes Dreke

% CT-SS Realization. Remark: Non-minimum phase system
Ac =  [0, 1; -6, -5];
Bc =  [0, 0.1; 1, 0];
Cc =  [-1, 1];

% DT-SS Realization
Ts =  0.1;                          % [s] Sampling time 
sysd = c2d(ss(Ac,Bc,Cc,[]),Ts);     % ZOH Discretization

% SS DT Matrices
A = sysd.A; B = sysd.B; C = sysd.C;


% Polytopic state and input constraints
Xbound = [-[1; 1], [1; 1]];            % [Xmax, Xmin] 
Ubound = [-[1; 1], [1; 1]];            % [Umax, Umin]  
Pvec = [0, 0.4, 0, 0.4, zeros(1,4)];    % Pvec = [1 x nxm]

% Matching controllers design
% Case 1: LQR 
Q = eye(2); R = 0.01*eye(2);
K_LQR = dlqr(A,B,Q,R);

% Case 2: Robust
[K_Robust,P_Robust]=getRobustK(A,B,[],Pvec,Xbound,Ubound,"robust");

% Case 3: Constrained
[K_Const,P_Const]=getRobustK(A,B,[],Pvec,Xbound,Ubound,"constrained");

% Case 4: Classic
[K_Class,P_Class]=getRobustK(A,B,[],Pvec,Xbound,Ubound,"classic");

%% Simulation
Tsim =  1000;
x_ss = zeros(2,1); u_ss =zeros(2,1);
x0 = [0.5; -0.5];

xvec_cell = cell(4,1);
uvec_cell = cell(4,1);

[xvec,uvec] = simSystemSateFeedBack(A,B,-K_LQR,x0,Tsim);
xvec_cell{1,1} = xvec;
uvec_cell{1,1} = uvec;
[xvec,uvec] = simSystemSateFeedBack(A,B,K_Class,x0,Tsim);
xvec_cell{2,1} = xvec;
uvec_cell{2,1} = uvec;
[xvec,uvec] = simSystemSateFeedBack(A,B,K_Robust,x0,Tsim);
xvec_cell{3,1} = xvec;
uvec_cell{3,1} = uvec;
[xvec,uvec] = simSystemSateFeedBack(A,B,K_Const,x0,Tsim);
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
