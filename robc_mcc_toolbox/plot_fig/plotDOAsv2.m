function Fig_DOA = plotDOAsv2(Pcell,legendHead)
% plotDOAs() function to plot a comparison between the LQR and
% RMC DOAs. Related to example 3.1 of [1]. 
% ==================================================================== %

Nk = size(Pcell,1);

cmap_line = winter(Nk);
cmap_back = cool(Nk);

% Creating Figure
Fig_DOA  = ieee_fig();

% Creating axes
ax = subplot(3,8,1:6); 

for i=1:Nk

    % Domain of attraction
    P = Pcell{i,1};

    % Variable
    x = sdpvar(2,1);

    % Creating plot objects with Yalmip
    hSx_Const = plot(x'*P*x<=1,[],'b') ;
    Xtx_Const = polyshape(hSx_Const{1,1}(1,:),hSx_Const{1,1}(2,:));

    % Ploting
    h = plot(Xtx_Const,'FaceColor',cmap_back(i,:),...
                'LineWidth',3,'FaceAlpha',0.5);
    hold(ax, "on")

    % Editing 
    h.LineStyle = '-';
    h.EdgeColor = cmap_line(i,:);
end


% Labeling and axes modifications
xlabel(ax,'$x_{1}$')
ylabel(ax,'$x_{2}$')

ax.XMinorGrid = 'on'; 
ax.YMinorGrid = 'on';

legend(ax,legendHead)
legend('Location','best')

% Save figure
% exportgraphics(ax,'paper_closedLoop_DOA.pdf',...
%     'BackgroundColor','none','ContentType','vector')

end