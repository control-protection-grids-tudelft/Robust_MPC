% example_tennet_open_loop_dynamics
% Author: Victor Daniel Reyes
% ---------------------------------------------------------------------- %
% Time parameters
Ts = 50e-6;     % [s] sampling time
% Load per-unit [p.u] MMC Parameters in a struct variable. 
MMC_ONSHORE  = getMMCAvgParameterPU('Tennet_2GW_Onshore', Ts);  % [p.u]
MMC_OFFSHORE = getMMCAvgParameterPU('Tennet_2GW_Offshore',Ts);  % [p.u]

% Building CT SS Representation: Onshore MMC
[Ac_on, Bc_on, Cc_on] = getPlantModel(MMC_ONSHORE.Rr, MMC_ONSHORE.Rac, ...
                            MMC_ONSHORE.Lr, MMC_ONSHORE.Lac, ...
                            MMC_ONSHORE.w, 'OCC');
% Building CT SS Representation: Offshore MMC
[Ac_of, Bc_of, Cc_of] = getPlantModel(MMC_OFFSHORE.Rr, MMC_OFFSHORE.Rac, ...
                            MMC_OFFSHORE.Lr, MMC_OFFSHORE.Lac, ...
                            MMC_OFFSHORE.w, 'OCC');
% DT SS Representation
SYS_On_DISC = c2d(ss(Ac_on, Bc_on, Cc_on,[]),MMC_ONSHORE.Ts_pu,'zoh');
SYS_Of_DISC = c2d(ss(Ac_of, Bc_of, Cc_of,[]),MMC_OFFSHORE.Ts_pu,'zoh');

A_on = SYS_On_DISC.A; B_on = SYS_On_DISC.B;   C_on = SYS_On_DISC.C;
A_of = SYS_Of_DISC.A; B_of = SYS_Of_DISC.B;   C_of = SYS_Of_DISC.C;

% Model uncertaint coefficient
NModel =  4;                    % Number of cases
% Edit this line to modify tolerance values ranges. Default: -0.5 to 0.5
prv_max =  [2e-2,...       % \Delta R_r,
            2e-2,...       % \Delta R_ac,
            0e-2,...       % \Delta L_r
            0e-2,...       % \Delta L_ac
            4e-1];         % \Delta w
prv = zeros(length(prv_max),NModel);  % Vector of uncertaint coefficient

for i=1:NModel
    for j=1:size(prv,1)
        prv(j,i)= (rand(1,1)-0.5)*prv_max(j);       %
    end
end
% ---------------------------------------------------------------------- %
%%
% Ploting section
Fig_poles = ieee_fig();

ax = subplot(3,4,[1:2,5:6]);
ax2 = subplot(3,4,3:4);
ax3 = subplot(3,4,7:8);
ax4 = subplot(3,4,9:12);

% Stable region definition
theta = linspace(0, 2*pi, 100);     % Points for the circle

% Fill function to plot circle (Green)
fill(ax, cos(theta), sin(theta), 'y', 'FaceAlpha', 0.4, ...
    'EdgeColor', 'k',LineWidth=2,LineStyle='--'); 
hold(ax,"on")   
% Unstable region definition
PLOT_LIM = 1.2;                     % Amplitude
fill(ax, [-PLOT_LIM PLOT_LIM PLOT_LIM -PLOT_LIM],...
    [-PLOT_LIM -PLOT_LIM PLOT_LIM PLOT_LIM], 'k', ...
                'FaceAlpha', 0.035, 'EdgeColor', 'k',LineWidth=2);

% Simulation time steps
Tsim =1e4;
% Initial conditions
x0 = [0; 0];
xss = [0.67; -1e-4]; 
% Steady-state signal computation
xvec = [x0, zeros(2,Tsim)];
uvec = zeros(2,Tsim+1);
cmap = hsv(NModel); 

for i=1:NModel    
    [Ac_on, Bc_on, Cc_on] = getPlantModel(MMC_ONSHORE.Rr+prv(1,i),...
                                            MMC_ONSHORE.Rac+prv(2,i), ...
                                            MMC_ONSHORE.Lr+prv(3,i),...
                                            MMC_ONSHORE.Lac+prv(4,i), ...
                                            MMC_ONSHORE.w+prv(5,i),...
                                            'OCC');

    SYS_On_DISC = c2d(ss(Ac_on, Bc_on, Cc_on,[]),MMC_ONSHORE.Ts_pu,'zoh');

    pzp =  pzplot(ax, SYS_On_DISC);         % Plot poles/zeros vectors
    pzp.Responses(i,:).MarkerSize = 22;     % Adjusts maker size
    pzp.Responses(i,:).Color = cmap(i,:);   % Adjusts color
    pzp.Responses(i,:).LineWidth = 3;       % Adjusts lineWidth

    for k=1:Tsim
        uvec(:,k)=(SYS_On_DISC.B\(eye(2)-SYS_On_DISC.A))*xss;
        xvec(:,k+1)=SYS_On_DISC.A*xvec(:,k)+SYS_On_DISC.B*uvec(:,k);      
    end

    plot(ax2,Ts*(0:Tsim)', xvec(1,:)','Color',cmap(i,:),'LineWidth',3)
    hold(ax2,"on") 
    plot(ax3,Ts*(0:Tsim)', xvec(2,:)','Color',cmap(i,:),'LineWidth',3)
    hold(ax3,"on") 
    scatter(ax4,i,prv(1,i),50, cmap(i,:),'LineWidth',3,'Marker','o')
    hold(ax4,"on") 
    scatter(ax4,i,prv(2,i),50, cmap(i,:),'LineWidth',3,'Marker','square')
    scatter(ax4,i,prv(5,i),50, cmap(i,:),'LineWidth',3,'Marker','diamond')
end

xlabel(ax2,'Time [s]'); xlabel(ax3,'Time [s]'); 
ylabel(ax2,'$i^{\Delta}_d$[p.u.]'); ylabel(ax3,'$i^{\Delta}_q$[p.u.]'); 
ylabel(ax4,'$\Delta R_{ac},\Delta R_r,\Delta \omega$')
xlabel(ax4,'$\sigma$')
legend(ax4)

legendHead =  cell(1,3*NModel);

for i=1:NModel
    legendHead{1,3*(i-1)+1} = ['$R_{ac}{-}\sigma:$',num2str(i)];
    legendHead{1,3*(i-1)+2} = ['$R_{r}{-}\sigma:$',num2str(i)];
    legendHead{1,3*(i-1)+3} = ['$\omega{-}\sigma:$',num2str(i)];
end
legend(ax4,legendHead)
legend(ax4,'Orientation','horizontal')
legend(ax4,'NumColumns',3)
legend(ax4, 'Location',	'eastoutside')
ax2.XMinorGrid = 'on'; ax2.YMinorGrid = 'on';
ax3.XMinorGrid = 'on'; ax3.YMinorGrid = 'on';
ax4.XMinorGrid = 'on'; ax4.YMinorGrid = 'on';

xlim(ax,[-PLOT_LIM PLOT_LIM])
ylim(ax,[-PLOT_LIM PLOT_LIM])
xlim(ax,[0.95 1.05])
ylim(ax,[-0.05 0.05])
ylim(ax2,[-PLOT_LIM PLOT_LIM])
ylim(ax3,[-PLOT_LIM PLOT_LIM])
title('Discrete-Time Pole-Zero Map');

% exportgraphics(gcf,'example_3_1_mmc_openLoop_inf.pdf','BackgroundColor','none','ContentType','vector')
% print('example_3_1_mmc_openLoop_inf', '-dpdf','-vector','-fillpage')