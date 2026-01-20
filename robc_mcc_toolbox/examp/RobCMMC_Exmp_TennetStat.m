% Example Tennet Station Control
% Author: Victor Daniel Reyes
% ---------------------------------------------------------------------- %
% Time parameters
clear all,
Ts = 50e-6;     % [s] sampling time
% Load per-unit [p.u] MMC Parameters in a struct variable. 
% Note: This function is within the examples folders
MMC_ONSHORE  = getMMCAvgParameterPU('Tennet_2GW_Onshore', Ts);  % [p.u]
MMC_OFFSHORE = getMMCAvgParameterPU('Tennet_2GW_Offshore',Ts);  % [p.u]

% Building CT SS Representation
% Onshore MMC
[Ac_on, Bc_on, Cc_on] = getPlantModel(MMC_ONSHORE.Rr, MMC_ONSHORE.Rac, ...
                            MMC_ONSHORE.Lr, MMC_ONSHORE.Lac, ...
                            MMC_ONSHORE.w, 'OCC');
% Offshore MMC
[Ac_of, Bc_of, Cc_of] = getPlantModel(MMC_OFFSHORE.Rr, MMC_OFFSHORE.Rac, ...
                            MMC_OFFSHORE.Lr, MMC_OFFSHORE.Lac, ...
                            MMC_OFFSHORE.w, 'OCC');
% DT SS Representation
SYS_On_DISC = c2d(ss(Ac_on, Bc_on, Cc_on,[]),MMC_ONSHORE.Ts_pu,'zoh');
SYS_Of_DISC = c2d(ss(Ac_of, Bc_of, Cc_of,[]),MMC_OFFSHORE.Ts_pu,'zoh');

A_on = SYS_On_DISC.A; B_on = SYS_On_DISC.B;   C_on = SYS_On_DISC.C;
A_of = SYS_Of_DISC.A; B_of = SYS_Of_DISC.B;   C_of = SYS_Of_DISC.C;

%% Simulations
Fig_poles = ieee_fig(); 
ax = subplot(3,3,[1:2,4:5]);
hold(ax,"on") 
% Define vertices of rectangle (e.g., from -10 to 10 on real, -10 to 10 imag)
PLOT_LIM = 1.2;
fill(ax, [-PLOT_LIM PLOT_LIM PLOT_LIM -PLOT_LIM],...
    [-PLOT_LIM -PLOT_LIM PLOT_LIM PLOT_LIM], 'r', ...
                'FaceAlpha', 0.1, 'EdgeColor', 'k',LineWidth=2);
theta = linspace(0, 2*pi, 100); % Points for the circle
fill(ax, cos(theta), sin(theta), 'g', 'FaceAlpha', 0.1, ...
    'EdgeColor', 'g',LineWidth=2,LineStyle=':'); % Green, semi-transparent

% Plot poles/zeros (e.g., from a system 'sys')
NModel =  20;
cmap = hsv(NModel);
prv = zeros(4,NModel);
for i=1:NModel
    prv(:,i)= (rand(4,1)-0.5)./[200, 100, 5, 5]';
    [Ac_on, Bc_on, Cc_on] = getPlantModel(MMC_ONSHORE.Rr+prv(1,i),...
                                            MMC_ONSHORE.Rac+prv(2,i),...
                                            MMC_ONSHORE.Lr+prv(3,i),...
                                            MMC_ONSHORE.Lac+prv(4,i),...
                                            MMC_ONSHORE.w,...
                                            'OCC');
    SYS_On_DISC = c2d(ss(Ac_on, Bc_on, Cc_on,[]),MMC_ONSHORE.Ts_pu,'zoh');

    sys_poles = pole(ss(SYS_On_DISC.A,...
                        SYS_On_DISC.B,...
                        SYS_On_DISC.C, [], MMC_ONSHORE.Ts_pu));

    sys_zeros = tzero(ss(SYS_On_DISC.A,...
                        SYS_On_DISC.B,...
                        SYS_On_DISC.C, [], MMC_ONSHORE.Ts_pu));    
    
    scatter(ax,real(sys_poles),imag(sys_poles),260,cmap(i,:),...
        'Marker','+','LineWidth',4)    
    if isempty(sys_zeros)==false
        scatter(ax,real(sys_poles),imag(sys_poles),260,cmap(i,:),...
            'Marker','o','LineWidth',4)   
    end
end
xlim(ax,[-PLOT_LIM PLOT_LIM])
ylim(ax,[-PLOT_LIM PLOT_LIM])
title('Discrete-Time Pole-Zero Map');

% ---------------------------------------------------------------------- %
% Open-loop simulation
% Initial conditions x0
Tsim =  1.4e4;
x0 = [0; 0];
xss = [0.67; -1e-4]; 
% Steady-state signal computation


xvec = [x0, zeros(2,Tsim)];
uvec = zeros(2,Tsim+1);

XCell =  cell(2,2);

for j=1:2    
    for k=1:Tsim
        if j==1
            uvec(:,k)=(B_on\(eye(2)-A_on))*xss;
            xvec(:,k+1)=A_on*xvec(:,k)+B_on*uvec(:,k);
        else
            uvec(:,k)=(B_of\(eye(2)-A_of))*xss;
            xvec(:,k+1)=A_of*xvec(:,k)+B_of*uvec(:,k);
        end       
    end
    XCell{j,1} = xvec;
    XCell{j,2} = uvec;
end


Fig_Open_Loop = ieee_fig();
ax = subplot(4,1,1);
plot(ax,Ts*(0:Tsim)',XCell{1,1}(1,:),LineWidth=2)
hold(ax,"on")
plot(ax,Ts*(0:Tsim)',XCell{2,1}(1,:),LineWidth=2)
hold(ax,"off")
ylabel(ax, '$i^{\Delta}_d$[p.u.]')
xlabel(ax, 'Time [s]')
ylim(ax, [0 1.4])
xlim(ax, 'tight')
legend(ax,'MMC Onshore', 'MMC Offshore')
legend(ax,'Location', 'eastoutside')
ax.XMinorGrid = "on"; ax.YMinorGrid = "on";

ax = subplot(4,1,2);
plot(ax,Ts*(0:Tsim-1)',XCell{1,2}(1,1:Tsim),LineWidth=2)
hold(ax,"on")
plot(ax,Ts*(0:Tsim-1)',XCell{2,2}(1,1:Tsim),LineWidth=2)
hold(ax,"off")
ylabel(ax, '$u^{\Delta}_d$[p.u.]')
xlabel(ax, 'Time [s]')
% ylim(ax, [0 1.4])
xlim(ax, 'tight')
legend(ax,'MMC Onshore', 'MMC Offshore')
legend(ax,'Location', 'eastoutside')
ax.XMinorGrid = "on"; ax.YMinorGrid = "on";

ax= subplot(4,1,3);
plot(ax,Ts*(0:Tsim)',XCell{1,1}(2,:),LineWidth=2)
hold(ax,"on")
plot(ax,Ts*(0:Tsim)',XCell{2,1}(2,:),LineWidth=2)
hold(ax,"off")
ylabel(ax, '$i^{\Delta}_q$[p.u.]')
xlabel(ax, 'Time [s]')
ylim(ax, [-0.7 0.6])
xlim(ax, 'tight')
legend(ax,'MMC Onshore', 'MMC Offshore')
legend(ax,'Location', 'eastoutside')
ax.XMinorGrid = "on"; ax.YMinorGrid = "on";

ax = subplot(4,1,4);
plot(ax,Ts*(0:Tsim-1)',XCell{1,2}(2,1:Tsim),LineWidth=2)
hold(ax,"on")
plot(ax,Ts*(0:Tsim-1)',XCell{2,2}(2,1:Tsim),LineWidth=2)
hold(ax,"off")
ylabel(ax, '$u^{\Delta}_q$[p.u.]')
xlabel(ax, 'Time [s]')
% ylim(ax, 'tight')
xlim(ax, 'tight')
legend(ax,'MMC Onshore', 'MMC Offshore')
legend(ax,'Location', 'eastoutside')
ax.XMinorGrid = "on"; ax.YMinorGrid = "on";


