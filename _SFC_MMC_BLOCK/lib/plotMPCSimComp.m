function plotMPCSimComp(xvecCell,uvecCell,x_ss,u_ss,Ts,Tsim,Nk)
    figure
    cmp = colormap(turbo(10));
    subplot(2,1,1)
    
    for i=1:Nk
        plot(Ts*[0:Tsim]',xvecCell{i,1}(3,:)', 'Color',cmp(i,:))
        hold on
    end
    plot(Ts*[0:Tsim-1]',x_ss(3,:)', '--k','LineWidth', 2.5)
    hold off
    xlim tight
    xlabel('Time [s]')
    ylabel('$y_1$')
    grid minor 
    
    
    subplot(2,1,2)
    
    for i=1:Nk
        plot(Ts*[0:Tsim]',xvecCell{i,1}(4,:)', 'Color',cmp(i,:))
        hold on
    end
    plot(Ts*[0:Tsim-1]',x_ss(4,:)', '--k','LineWidth', 2.5)
    hold off
    xlim tight
    xlabel('Time [s]')
    ylabel('$y_2$')
    grid minor 
    
    figure 
    cmp = colormap(turbo(10));
    subplot(2,1,1)
    
    for i=1:Nk
        plot(Ts*[0:Tsim-1]',uvecCell{i,1}(1,:)', 'Color',cmp(i,:))
        hold on
    end
    plot(Ts*[0:Tsim-1]',u_ss(1,:)', '--k','LineWidth', 2.5)
    hold off
    xlim tight
    xlabel('Time [s]')
    ylabel('$\Delta u_1$')
    grid minor 
    
    
    subplot(2,1,2)
    cmp = colormap(turbo(10));
    for i=1:Nk
        plot(Ts*[0:Tsim-1]',uvecCell{i,1}(2,:)', 'Color',cmp(i,:))
        hold on
    end
    plot(Ts*[0:Tsim-1]',u_ss(1,:)', '--k','LineWidth', 2.5)
    hold off
    xlim tight
    xlabel('Time []')
    ylabel('$\Delta u_2$')
    grid minor 
end