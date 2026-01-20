function Fig_poles=plotAugFullInformationClosed(MMC_ONSHORE,K,prv,Tsim,...
    x0,xss,w)
% plotAugFullInformationClosed()
% ---------------------------------------------------------------------- %
% Nominal model 
[Ac0_on, Bc0_on, Cc0_on] = getPlantModel(MMC_ONSHORE.Rr,...
                                            MMC_ONSHORE.Rac, ...
                                            MMC_ONSHORE.Lr,...
                                            MMC_ONSHORE.Lac, ...
                                            MMC_ONSHORE.w,...
                                            'OCC');

PLOT_LIM = 1.2;                     % Amplitude

NModel = size(prv,2);
if MMC_ONSHORE.Ts*Tsim<=0.01
    timeLine = 1e3*MMC_ONSHORE.Ts*(0:Tsim)';
    timeHead = 'Time [ms]';
else
    timeLine = 1e0*MMC_ONSHORE.Ts*(0:Tsim)';
    timeHead = 'Time [s]';
end
% ---------------------------------------------------------------------- %
Fig_poles = ieee_fig();

ax = subplot(3,3,1:2);      % poles map
ax2 = subplot(3,3,4:5);     % states i_d
ax3 = subplot(3,3,7:8);     % states i_q
ax4 = subplot(3,3,3);       % Uncertain values
ax5 = subplot(3,3,6);       % input u_d
ax6 = subplot(3,3,9);       % input u_q


% Stable region definition
theta = linspace(0, 2*pi, 100);     % Points for the circle

% Fill function to plot circle (Green)
fill(ax, cos(theta), sin(theta), 'y', 'FaceAlpha', 0.4, ...
    'EdgeColor', 'k',LineWidth=2,LineStyle='--'); 
hold(ax,"on")   

% Unstable region definition
fill(ax, [-PLOT_LIM PLOT_LIM PLOT_LIM -PLOT_LIM],...
    [-PLOT_LIM -PLOT_LIM PLOT_LIM PLOT_LIM], 'k', ...
                'FaceAlpha', 0.035, 'EdgeColor', 'k',LineWidth=2);


% Steady-state signal computation
xvec = [x0, zeros(size(x0,1),Tsim)];
uvec = zeros(size(Bc0_on,2),Tsim+1);
ussvec = zeros(size(Bc0_on,2),Tsim+1);
cmap = hsv(NModel); 

for i=1:NModel    
    [Ac_on, Bc_on, Cc_on] = getPlantModel(MMC_ONSHORE.Rr+prv(1,i),...
                                            MMC_ONSHORE.Rac+prv(2,i), ...
                                            MMC_ONSHORE.Lr+prv(3,i),...
                                            MMC_ONSHORE.Lac+prv(4,i), ...
                                            MMC_ONSHORE.w+prv(5,i),...
                                            'OCC');
    
    % Open-loop
    SYS_On_DISC = c2d(ss(Ac_on, Bc_on, Cc_on,[]),...
                                MMC_ONSHORE.Ts_pu,'zoh');
    SYS_On_Nom_DISC = c2d(ss(Ac0_on, Bc0_on, Cc0_on,[]),...
                                MMC_ONSHORE.Ts_pu,'zoh');
     % Delta continous-time parameters
    DeltaA =  SYS_On_DISC.A - SYS_On_Nom_DISC.A;
    DeltaB =  SYS_On_DISC.B - SYS_On_Nom_DISC.B;
    % DeltaC =  SYS_On_DISC.C - SYS_On_Nom_DISC.C;

    [A_on, B_on, C_on] = getAugmentedSystem(SYS_On_DISC.A, ...
                                            SYS_On_DISC.B, ...
                                            SYS_On_DISC.C);
    % Closed-loop
    SYS_Clo_DISC = ss(A_on+B_on*K, 0*B_on, C_on,[],MMC_ONSHORE.Ts_pu);

    sys_poles = pole(SYS_Clo_DISC);
    sys_zeros = tzero(SYS_Clo_DISC);    
    
    scatter(ax,real(sys_poles),imag(sys_poles),260,cmap(i,:),...
        'Marker','+','LineWidth',4)
    
    if isempty(sys_zeros)==false
        scatter(ax,real(sys_poles),imag(sys_poles),260,cmap(i,:),...
            'Marker','o','LineWidth',4)   
    end

    for k=1:Tsim
        ussvec(:,k)=(B_on\(eye(size(x0,1))-A_on))*xss(:,k);
        uvec(:,k)= min(2.0, max(-2.0, K*(xvec(:,k)-xss(:,k))+ussvec(:,k)));
        xvec(:,k+1)=A_on*xvec(:,k)+B_on*uvec(:,k)+w(:,k);      
    end

    plot(ax2,timeLine, xvec(1,:)',...
        'Color',cmap(i,:),'LineWidth',3)
    hold(ax2,"on") 
    plot(ax2,timeLine, xvec(2,:)','--',...
        'Color',cmap(i,:),'LineWidth',3)
    plot(ax3,timeLine, xvec(3,:)',...
        'Color',cmap(i,:),'LineWidth',3)
    hold(ax3,"on") 
    plot(ax3,timeLine, xvec(4,:)','--',...
        'Color',cmap(i,:),'LineWidth',3) 

    plot(ax5,timeLine, uvec(1,:)','Color',cmap(i,:),'LineWidth',3)
    hold(ax5,"on") 
    plot(ax6,timeLine, uvec(2,:)','Color',cmap(i,:),'LineWidth',3)
    hold(ax6,"on") 
    scatter(ax4,i,DeltaA(1,1),50, cmap(i,:),...
        'LineWidth',3,'Marker','o')
    hold(ax4,"on")
    scatter(ax4,i,DeltaA(1,2),50, cmap(i,:),...
        'LineWidth',3,'Marker','square')     
    scatter(ax4,i,DeltaB(1,1),50, cmap(i,:),...
        'LineWidth',3,'Marker','diamond')
    
end
plot(ax2,timeLine, xss(1:2,:)','--k','LineWidth',2.2)
plot(ax3,timeLine, xss(3:4,:)','--k','LineWidth',2.2)

title(ax,'Zeros-poles map');
xlabel(ax,'Real-axis'); ylabel(ax,'Imag-axis'); 
xlabel(ax6,timeHead); xlabel(ax3,timeHead); 
ylabel(ax2,'$i^{\Delta}_d$[p.u.]'); ylabel(ax3,'$i^{\Delta}_q$[p.u.]');
ylabel(ax5,'$u^{\Delta}_d$[p.u.]'); ylabel(ax6,'$u^{\Delta}_q$[p.u.]'); 

xlabel(ax4,'$\sigma$')
ylabel(ax4,'$\rho(\sigma)$')

legendHead =  cell(1,2+NModel);
legendHead{1,1} = 'Stable region';
legendHead{1,2} = 'Unstable region';

for i=1:NModel
    legendHead{1,2+i} = ['$\sigma:$',num2str(i),' '];
end

legend(ax,legendHead)
legend(ax,'NumColumns',3)
legend(ax, 'Location','north')

legend(ax4,{'$\rho_{1,4}$','$\rho_{2,3}$','$\rho_{5,6}$'})
legend(ax4, 'Location',	'best')

ax2.XMinorGrid = 'on'; ax2.YMinorGrid = 'on';
ax3.XMinorGrid = 'on'; ax3.YMinorGrid = 'on';
ax4.XMinorGrid = 'on'; ax4.YMinorGrid = 'on';
ax5.XMinorGrid = 'on'; ax5.YMinorGrid = 'on';
ax6.XMinorGrid = 'on'; ax6.YMinorGrid = 'on';

xlim(ax,[-PLOT_LIM PLOT_LIM])
ylim(ax,[-PLOT_LIM PLOT_LIM])
ylim(ax2,'tight')
ylim(ax3,'tight')
xlim(ax5,'tight')
xlim(ax6,'tight')
xlim(ax2,'tight')
xlim(ax3,'tight')


end