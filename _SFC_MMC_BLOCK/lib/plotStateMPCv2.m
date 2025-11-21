function FigMain=plotStateMPCv2(XUvec,Nk,Tsim,r,tShow,tIni)
% FigMain=plotStateMPC(XUvec,Nk,Tsim,r,tShow,tIni)
% Date: 03-10-2025
% ---------------------------------------------------------------------- %
% Authors: (i) Victor Daniel Reyes Dreke.
% ====================================================================== %
cmap = cool(Nk); 
cNom = 'g';
cMin = cmap(1,:);
cMax = cmap(Nk,:);

timeLine = (0:1:Tsim)';

FigMain = figure();

ax1 = subplot(3,1,1);
ax2 = subplot(3,1,2);


for i=1:Nk
    x = XUvec{i,1};

    if i==1
        % Nominal case
        lnW = 2;
        plot(ax1,timeLine, x(1,:)', 'LineWidth',lnW,...
                                    'Color',cNom,...
                                    'DisplayName', 'nom')
                ylabel(ax1,'$\Delta \iota^{\Delta}_{d}(k)$')
                hold(ax1, "on")
        plot(ax2,timeLine, x(2,:)', 'LineWidth',lnW,...
                                    'Color',cNom,...
                                    'DisplayName', 'nom')
                ylabel(ax2,'$\Delta \iota^{\Delta}_{q}(k)$')
                hold(ax2, "on")  

        % Minimum  value
        xmin = XUvec{2,1};
        plot(ax1,timeLine , xmin(1,:)','LineWidth',3,...
            'Color',cMin,'DisplayName', 'min')
        plot(ax2,timeLine , xmin(2,:)','LineWidth',3,...
            'Color',cMin,'DisplayName', 'min')

        % Maximum value
        xmax = XUvec{Nk,1};
        plot(ax1,timeLine , xmax(1,:)','LineWidth',3,...
            'Color',cMax,'DisplayName', 'max')
        plot(ax2,timeLine , xmax(2,:)','LineWidth',3,...
            'Color',cMax,'DisplayName', 'max')

    else
        plot(ax1,timeLine , x(1,:)','LineWidth',2.5,'Color',cmap(i,:))
        plot(ax2,timeLine , x(2,:)','LineWidth',2.5,'Color',cmap(i,:))


    end
end 

% Final plots 
x = XUvec{1,1};
xmin = XUvec{2,1};
xmax = XUvec{Nk,1};



lnW =2;
plot(ax1,timeLine , xmin(1,:)','LineWidth',lnW,'Color',cMin)
plot(ax2,timeLine , xmin(2,:)','LineWidth',lnW,'Color',cMin)

plot(ax1,timeLine , xmax(1,:)','LineWidth',lnW,'Color',cMax)
plot(ax2,timeLine , xmax(2,:)','LineWidth',lnW,'Color',cMax)

plot(ax1,timeLine , x(1,:)','LineWidth',lnW,'Color',cNom)
plot(ax2,timeLine , x(2,:)','LineWidth',lnW,'Color',cNom)

timeLine = (0:1:Tsim-1)';
stairs(ax1,timeLine , r(1,1:Tsim)','--k','LineWidth',1.5,'DisplayName', 'ref')
stairs(ax2,timeLine , r(2,1:Tsim)','--k','LineWidth',1.5,'DisplayName', 'ref')

command_1 = 'colormap(ax1, cool);'; % check autumn
for i=1:2    
    command_1(12) = num2str(i);
    eval(command_1);
end

for i=1:2
    command_2 = "c = colorbar(ax"+string(i)+",'Direction','normal');";
    eval(command_2)
    c.TickLabelInterpreter = 'latex';
    c.Label.Interpreter = 'latex';
    c.Label.String = '$\Delta L$';
    c.Limits = [0 1];
    c.TickLabels = {'$-L_m$','0','$L_m$'};
end



 command_expr = 'xlim(ax1, [tIni tShow])';
 for i=1:2     
     command_expr(8) = num2str(i);
     eval(command_expr)
 end

 ylim(ax1,[-2 2])
 ylim(ax2,[-2 2])

 xlabel(ax2, 'Time step $[k]$')

 
 ax1.XMinorGrid = 'on'; ax1.YMinorGrid = 'on';
 ax2.XMinorGrid = 'on'; ax2.YMinorGrid = 'on';



  legend(ax2,{'$\Delta L_g =0$',...
     '$\Delta L_g =-L_{g}$',...
     '$\Delta L_g =L_{g}$'},...
     'Location','southwest', 'Orientation','vertical')
end