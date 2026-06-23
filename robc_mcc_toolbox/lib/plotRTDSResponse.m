function plotRTDSResponse(fileNames,plotFlag,rho)
% plotRTDSResponse(fileNames,plotFlag,rho)

% Table layout
%%% =========================== %%%
Fig = ieee_fig;
numPlot = 6;
tF = tiledlayout(numPlot,1);
%%% =========================== %%%

%%% =========================== %%%

% Ploting figure
%%% =========================== %%%
for i=1:numPlot
    ax = nexttile(i);
    switch i
        case 1 % Plot DC voltage
            datap = [];
            xLabelHead = 'Time [s]';
            yLabelHead = '$V_{dc}$ [kV]';
            legendHead = {'MMC.1','MMC.2'};
            for j=1:2
                fileNameTemp = [fileNames,'_',num2str(i-1+j-1),'.csv'];
                tempT = readtable(fileNameTemp);
                tempT = table2array(tempT);
                if j==1
                    datap = [datap, tempT];
                else
                    datap = [datap, tempT(:,2)];
                end
            end
        case 2 % Plot i_dc
            datap = [];
            xLabelHead = 'Time [s]';
            yLabelHead = '$i_{dc}$ [kA]';
            legendHead = {'$i_{dc}$:MMC.1'};
            fileNameTemp = [fileNames,'_',num2str(i),'.csv'];
            tempT = readtable(fileNameTemp);
            tempT = table2array(tempT);
            datap = [datap, tempT];
        case 3 % Plot Grid currents
            datap = [];
            xLabelHead = 'Time [s]';
            yLabelHead = '$i^{G1}_{abc}$ [kA]';
            legendHead = {'$i^{G1}_{a}$','$i^{G1}_{b}$','$i^{G1}_{c}$'};
            for j=1:3
                fileNameTemp = [fileNames,'_',num2str(i+j),'.csv'];
                tempT = readtable(fileNameTemp);
                tempT = table2array(tempT);
                if j==1
                    datap = [datap, tempT];
                else
                    datap = [datap, tempT(:,2)];
                end
            end
        case 4 % Plot Grid voltage
            datap = [];
            xLabelHead = 'Time [s]';
            yLabelHead = '$i^{G1}_{abc}$ [kV]';
            legendHead = {'$v^{G1}_{a}$','$v^{G1}_{b}$','$v^{G1}_{c}$'};
            for j=1:3
                fileNameTemp = [fileNames,'_',num2str(i+2+j),'.csv'];
                tempT = readtable(fileNameTemp);
                tempT = table2array(tempT);
                if j==1
                    datap = [datap, tempT];
                else
                    datap = [datap, tempT(:,2)];
                end
            end
        case 5 % Plot Grid voltage
            datap = [];
            xLabelHead = 'Time [s]';
            yLabelHead = '$P_{ac}$ [MW]';
            legendHead = {'MMC.1'};
            fileNameTemp = [fileNames,'_',num2str(i+5),'.csv'];
            tempT = readtable(fileNameTemp);
            tempT = table2array(tempT);
            datap = [datap, tempT];
        case 6
            datap = [];
            xLabelHead = 'Time [s]';
            yLabelHead = '$Q_{ac}$ [MVA]';
            legendHead = {'MMC.1'};
            fileNameTemp = [fileNames,'_',num2str(i+5),'.csv'];
            tempT = readtable(fileNameTemp);
            tempT = table2array(tempT);
            datap = [datap, tempT];
    end
    % tempT = table2array(tempT);
    % tempT = tempT(:,1:2);
    plot(ax,datap(:,1), datap(:,2:end));
    xlabel(ax,xLabelHead);
    ylabel(ax,yLabelHead);
    legend(ax,legendHead);
    ax.XMinorGrid="on"; ax.YMinorGrid="on";
end
    
end