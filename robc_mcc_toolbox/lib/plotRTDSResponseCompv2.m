function Fig =plotRTDSResponseCompv2(numPlot,fileNames,varId,...
    xLabelHead,yLabelHead,legendHead)
% plotRTDSResponse(fileNames,plotFlag,rho)

% Table layout
%%% =========================== %%%
Fig = ieee_fig;
numCases = size(fileNames,1);
tF = tiledlayout(numPlot+3,3);
tF.Padding = 'compact';
tF.TileSpacing = 'compact';
%%% =========================== %%%

%%% =========================== %%%

% Ploting figure
%%% =========================== %%%
for i=1:numPlot        
    for j=1:numCases
        ax = nexttile(1,[1:2]);
        ax2 = nexttile(3);
        datap = [];
        for k=1:length(varId)
            fileNameTemp = [fileNames{j,1},num2str(varId(k)),'.csv'];
            tempT = readtable(fileNameTemp);
            tempT = table2array(tempT);
            if k==1
                datap = [datap, tempT];
            else
                datap = [datap, tempT(:,2)];
            end
        end
        plot(ax,datap(:,1), datap(:,2:end),'LineWidth',3);
        hold(ax,"on")
        xlabel(ax,xLabelHead);
        ylabel(ax,yLabelHead);
        xlim(ax,[0 2])
        lgd = legend(ax,legendHead);
        lgd.Location ='best';
        ax.XMinorGrid="on"; ax.YMinorGrid="on";

        plot(ax2,datap(:,1), datap(:,2:end),'LineWidth',3);
        hold(ax2,"on")
        xlabel(ax2,xLabelHead);
        % ylabel(ax2,yLabelHead);
        xlim(ax2,[1.5 1.6])
        % lgd2 = legend(ax2,legendHead);
        % lgd2.Location ='best';
        ax2.XMinorGrid="on"; ax2.YMinorGrid="on";
    end 
    % if varId~=10||varId~=11
        linkaxes([ax,ax2],"y")
    % end
end


    
end