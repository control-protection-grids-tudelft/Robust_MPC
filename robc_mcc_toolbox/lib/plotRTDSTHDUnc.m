function Fig = plotRTDSTHDUnc(fileNames,plotFlag,rho,Ts,freqInt)
 % plotRTDSTHDUnc()
    % fileNames = 'C:\Users\vreyesdreke\Documents\DATA_EXP\RMPC_JOURNAL\TRANSIENT_RMPC\RMPC_STEADY_TEST_TIME_';
    % fileNames2 = 'C:\Users\vreyesdreke\Documents\DATA_EXP\RMPC_JOURNAL\TRANSIENT_LQR\LQR_STEADY_TEST_TIME_';
    % Data initialization
    %%% =========================== %%%
    dataA = [];
    freqA = [];

    % Table layout
    %%% =========================== %%%
    Fig = ieee_fig;
    colormap(parula);
    numPlot = 1;
    tF = tiledlayout(numPlot+1,3);
    
    % Data storage
    %%% =========================== %%%
    for i=1:length(rho)
        % fileNames = 'C:\Users\vreyesdreke\Documents\DATA_EXP\RMPC_JOURNAL\TRANSIENT_RMPC\RMPC_STEADY_TEST_TIME_';
        tempT = readtable([fileNames,num2str(i+1),'_5.csv']);
        tempT = table2array(tempT);
        % tempT = tempT(:,2);
        [freqX,Pfft_x] = x2FFT2(tempT(:,2),Ts);
        dataA = [dataA, Pfft_x];
        freqA = [freqA, freqX];
    end
    ax = nexttile(1,[2,2]);
    imagesc(ax,freqA(freqInt,1)/1000,rho, dataA(freqInt,:)')
    set(ax, 'YDir', 'normal');
    hold(ax,"on");
    for j=1:length(rho) 
        xlinHead = ['$(L_r,R_r)=$', num2str(rho(j)),'(L_{r0},R_{r0})'];
        labels = ['$\alpha=$', num2str(rho(j))];
        yline(ax,rho(j),'-',labels,...
            'Color',[0.0 0.0 0.0],...
            'LineWidth',1.5,'DisplayName',xlinHead,...
            'Interpreter','latex','FontSize',20,...
            'LabelHorizontalAlignment','right','LabelVerticalAlignment','middle');
    end
    xl = xline(ax,50/1000,'-','$f_0 =50$Hz',...
        'Color',[0.7 0.7 0.7],'LineWidth',5.5,...
        'DisplayName','Nominal Freq.');    
    xl.LabelVerticalAlignment = 'middle';
    xl.LabelHorizontalAlignment = 'center';
    xl.Interpreter=	"latex";
    xl.FontSize = 20;

    ylabel(ax,'$\alpha$')
    xlabel(ax,'Frequency [kHz]')

    c = colorbar;
    c.TickLabelInterpreter = 'latex';
    c.Label.Interpreter = 'latex';
    c.Label.String = 'THD$(f)$ [kA]';
    % c.Limits = [0 1];
    % c.Ticks = ([0.1, 0.5, 1, 2, 3]-0.1)/2.9;
    % c.TickLabels ={'0.1','0.5', '1','2', '3'};
    % c.Layout.Tile = 'east';

end 

