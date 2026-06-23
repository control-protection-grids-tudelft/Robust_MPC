function Fig = plotUncertaintEigMMC(A,B,Arho,Brho,rho,K)
% Fig = plotUncertaintEig(A,B,Arho,Brho,K) 
% This plot is tailored to plot a set of eigenvalues corresponding to set
% of uncertainties realizations. 
% Author: Victor Daniel Reyes Dreke. 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Note: This is not an efficient function. 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

nl = length(abs(eig(A+B*K)));
N = length(rho);
nx = size(A,1);
nu = size(B,2);


lambVec = zeros(nl,N);

for k=1:N
    lambVec(:,k) = eig(Arho(nx*(k-1)+1:nx*k,:)+Brho(nx*(k-1)+1:nx*k,:)*K);
end
lambda0 = eig(A+B*K);

Fig =ieee_fig;

% for k=1:nl
    % ax = subplot(nl,1,k);

    ax = subplot(2,2,1); 

    % ============ Plot regions
    % Stable region definition
    theta = linspace(0, 2*pi, 200);
    x1 = cos(theta);
    y1 = sin(theta);
    
    % Unstable region
    x2 = [-5, -5, 5, 5]*20;
    y2 = [5, -5, -5, 5]*20;
    pgon = polyshape({x1,x2},{y1,y2});
    % ============
    
    % Scatter parameters
    sz = 75;
    cmap = parula(N); 
    plot(ax,pgon, 'FaceAlpha', 0.6,...
                'FaceColor',[1 0 0],...
                'LineWidth',3,...
                'LineStyle','--')
    hold(ax,"on")
    scatter(ax,real(lambVec),imag(lambVec),...
        sz,cmap,'*','LineWidth',2.0)
    % Original
    % scatter(ax,real(lambVec),imag(lambVec),...
    %     sz,cmap,'filled')
    scatter(ax,real(lambda0),imag(lambda0),...
        sz*1.5,[1, 0, 0],'+','LineWidth',2.5)
    c = colorbar(ax,'Direction','normal');
    c.TickLabelInterpreter = 'latex';
    c.Label.Interpreter = 'latex';
    c.Label.String = '$\alpha$';
    c.Limits = [0 1];
    c.Ticks = ([0.1, 0.5, 1, 2, 3]-0.1)/2.9;
    c.TickLabels ={'0.1','0.5', '1','2', '3'};

    xlim(ax,[-2,1.1]);
    ylim(ax,[-1.1,1.1]);
    
    xlabel(ax,'Re$(\lambda)$');
    ylabel(ax,'Img$(\lambda)$');
    
    ax.XMinorGrid="on"; ax.YMinorGrid="on";
% end

end