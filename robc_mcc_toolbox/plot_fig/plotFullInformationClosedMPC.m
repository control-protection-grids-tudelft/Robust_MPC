function Fig_poles=plotFullInformationClosedMPC(MMC_ONSHORE,...
    Q,R,P,N,Xbound,Ubound,prv,Tsim,...
    x0,xss,w)
% plotFullInformationClosedMPC( )
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

ax2 = subplot(3,3,4:5);     % states i_d
ax3 = subplot(3,3,7:8);     % states i_q
ax5 = subplot(3,3,6);       % input u_d
ax6 = subplot(3,3,9);       % input u_q

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
    
    
    % Simulate MPC
    ussvec = (SYS_On_DISC.B\(eye(2)-SYS_On_DISC.A))*xss;
    [xvec,uvec]= simMPCwRealPlant(SYS_On_Nom_DISC.A,...
                                    SYS_On_Nom_DISC.B,...
                                    SYS_On_DISC.A,...
                                    SYS_On_DISC.B,...
                                    Q,P,R,N,x0,...
                                    Xbound,Ubound,...
                                    xss,ussvec,Tsim,false,w,'input_rate');

    plot(ax2,timeLine, xvec(1,:)',...
        'Color',cmap(i,:),'LineWidth',3)
    hold(ax2,"on") 
    plot(ax3,timeLine, xvec(2,:)',...
        'Color',cmap(i,:),'LineWidth',3)
    hold(ax3,"on") 
    plot(ax5,timeLine, uvec(1,:)','Color',cmap(i,:),'LineWidth',3)
    hold(ax5,"on") 
    plot(ax6,timeLine, uvec(2,:)','Color',cmap(i,:),'LineWidth',3)
    hold(ax6,"on") 
    
end
plot(ax2,timeLine, xss(1,:)','--k','LineWidth',2.2)
plot(ax3,timeLine, xss(2,:)','--k','LineWidth',2.2)

xlabel(ax6,timeHead); xlabel(ax3,timeHead); 
ylabel(ax2,'$i^{\Delta}_d$[p.u.]'); ylabel(ax3,'$i^{\Delta}_q$[p.u.]');
ylabel(ax5,'$u^{\Delta}_d$[p.u.]'); ylabel(ax6,'$u^{\Delta}_q$[p.u.]'); 


legendHead =  cell(1,NModel);
for i=1:NModel
    legendHead{1,i} = ['$\sigma:$',num2str(i),' '];
end

legend(ax2,legendHead)
legend(ax2,'NumColumns',2)
legend(ax2, 'Location','best')

ax2.XMinorGrid = 'on'; ax2.YMinorGrid = 'on';
ax3.XMinorGrid = 'on'; ax3.YMinorGrid = 'on';
ax5.XMinorGrid = 'on'; ax5.YMinorGrid = 'on';
ax6.XMinorGrid = 'on'; ax6.YMinorGrid = 'on';

ylim(ax2,'tight')
ylim(ax3,'tight')
xlim(ax5,'tight')
xlim(ax6,'tight')
xlim(ax2,'tight')
xlim(ax3,'tight')


end