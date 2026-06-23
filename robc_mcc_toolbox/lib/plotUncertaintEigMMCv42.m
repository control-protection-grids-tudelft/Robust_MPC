function Fig = plotUncertaintEigMMCv42(A,B,Arho,Brho,rho,K,K2)
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
lambVec2 = zeros(nl,N);

for k=1:N
    lambVec(:,k) = eig(Arho(nx*(k-1)+1:nx*k,:)+Brho(nx*(k-1)+1:nx*k,:)*K);
end
lambda0 = eig(A+B*K);

for k=1:N
    lambVec2(:,k) = eig(Arho(nx*(k-1)+1:nx*k,:)+Brho(nx*(k-1)+1:nx*k,:)*K2);
end
lambda02 = eig(A+B*K2);

Fig =ieee_fig;
taxiss = tiledlayout(3,3);

% ============================
% ============ Plot regions
% Stable region definition
theta = linspace(0, 2*pi, 200);
x1 = cos(theta);
y1 = sin(theta);

% Unstable region
x2 = [-5, -5, 5, 5]*20;
y2 = [5, -5, -5, 5]*20;
pgon = polyshape({x1,x2},{y1,y2});

% ============================
% ============ Scatter parameters

sz = 75;
cmap = parula(N); 
for j=1:2
    if j==1
        ax = nexttile([1,2]);
        ax2 = nexttile(3);
        lambVecT = lambVec;
        lambda0T = lambda0;
    else
        ax = nexttile(4,[1 2]);
        ax2 = nexttile(6);
        lambVecT = lambVec2;
        lambda0T = lambda02;
    end

    plot(ax,pgon, 'FaceAlpha', 0.6,...
                    'FaceColor',[1 0 0],...
                    'LineWidth',3,...
                    'LineStyle','--');
    hold(ax,"on");
    scatter(ax,real(lambVecT),imag(lambVecT),...
            sz,cmap,'*','LineWidth',2.0);
    scatter(ax,real(lambda0T),imag(lambda0T),...
            sz*1.5,[1, 0, 0],'+','LineWidth',2.5);
    
    xlabel(ax,'Re$(\lambda)$');
    ylabel(ax,'Img$(\lambda)$');
    
    % ============ Limits
    xlim(ax,[-1.1,1.1]);
    ylim(ax,[-1.1,1.1]);
        
    ax.XMinorGrid="on"; ax.YMinorGrid="on";   
    
    % ============================ 
    
    % ============================
    % ============ Zoming plot
    % Stable region definition
    plot(ax2,pgon, 'FaceAlpha', 0.6,...
                    'FaceColor',[1 0 0],...
                    'LineWidth',3,...
                    'LineStyle','--');
    hold(ax2,"on");
    scatter(ax2,real(lambVecT),imag(lambVecT),...
            sz,cmap,'*','LineWidth',2.0);
    scatter(ax2,real(lambda0T),imag(lambda0T),...
            sz*1.5,[1, 0, 0],'+','LineWidth',2.5);
    
    xlabel(ax2,'Re$(\lambda)$');
    % ylabel(ax,'Img$(\lambda)$');
    
    % ============ Limits
    if j==1
        xlim(ax2,[0.98,1.02]);
        ylim(ax2,[-0.03,0.03]);
    else
        xlim(ax2,[0.6,1.1]);
        ylim(ax2,[-0.1,0.1]);
    end    
        
    ax2.XMinorGrid="on"; ax2.YMinorGrid="on";  
end
    

% c = colorbar(ax,'Direction','normal');
% ============================ 
% Colorbar
c = colorbar;
c.TickLabelInterpreter = 'latex';
c.Label.Interpreter = 'latex';
c.Label.String = '$\alpha$';
c.Limits = [0 1];
c.Ticks = ([0, 0.5, 1, 2, 3]-0.1)/2.9;
c.TickLabels ={'0.1','0.5', '1','2', '3'};
c.Layout.Tile = 'north';


end