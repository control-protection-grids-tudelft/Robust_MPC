function Fig_DOA = plotDOAs(P_Const,P_LQR)
% plotDOAs() function to plot a comparison between the LQR and
% RMC DOAs. Related to example 3.1 of [1]. 
% ==================================================================== %
% Color Palettes:  
% URL:https://www.heavy.ai/blog/12-color-palettes-for-telling-better-stories-with-your-data
% Blue to Red (Default)
blue2redColors = ["#1984c5", "#22a7f0", "#63bff0", "#a7d5ed", ...
    "#e2e2e2", "#e1a692", "#de6e56", "#e14b31", "#c23728"];
% Orange to Purple
orange2purpleColors = ["#ffb400", "#d2980d", "#a57c1b", "#786028",...
    "#363445", "#48446e", "#5e569b", "#776bcd", "#9080ff"];
% Pink Foam
pink2foamColors = ["#54bebe", "#76c8c8", "#98d1d1", "#badbdb",...
    "#dedad2", "#e4bcad", "#df979e", "#d7658b", "#c80064"];
% Salmon to Aqua
salmon2aquaColors = ["#e27c7c", "#a86464", "#6d4b4b", "#503f3f",...
    "#333333", "#3c4e4b", "#466964", "#599e94", "#6cd4c5"];
% ==================================================================== %
x = sdpvar(2,1);

% Creating plot objects with Yalmip
hSx_Const = plot(x'*P_Const*x<=1,[],'b') ;
hSx_LQR = plot(x'*P_LQR*x<=1,[],'b') ;

% Creating a polyshape objects
Xtx_Const = polyshape(hSx_Const{1,1}(1,:),hSx_Const{1,1}(2,:));
Xtx_LQR = polyshape(hSx_LQR{1,1}(1,:),hSx_LQR{1,1}(2,:));

% Creating Figure
Fig_DOA  = ieee_fig();

% Creating axes
ax = subplot(3,8,1:6); 

% Ploting
h = plot(Xtx_Const,'FaceColor','#76c8c8','lineWidth',3,'FaceAlpha',0.9);
hold(ax, "on")
h2 = plot(Xtx_LQR,'FaceColor','#df979e','lineWidth',3,'FaceAlpha',0.9);

% Editing 
h.LineStyle = '-';
h.EdgeColor = '#54bebe';
h2.EdgeColor = '#c80064';

% Labeling and axes modifications
xlabel(ax,'$x^{\Delta}_{d}$')
ylabel(ax,'$x^{\Delta}_{q}$')
ax.XMinorGrid = 'on'; ax.YMinorGrid = 'on';
legend(ax, {'DOA: RMC','DOA: LQR'})

% Save figure
% exportgraphics(ax,'paper_closedLoop_DOA.pdf',...
%     'BackgroundColor','none','ContentType','vector')

end