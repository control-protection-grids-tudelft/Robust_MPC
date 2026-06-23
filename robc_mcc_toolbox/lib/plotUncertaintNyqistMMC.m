function Fig = plotUncertaintNyqistMMC(A,B,C,Arho,Brho,Crho,rho,K)
% Fig = plotUncertaintNyqistMMC(A,B,Arho,Brho,K) 
% This plot is tailored to plot a set of eigenvalues corresponding to set
% of uncertainties realizations. 
% Author: Victor Daniel Reyes Dreke. 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Note: This is not an efficient function. 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% nl = length(abs(eig(A+B*K)));
N = length(rho);
nx = size(A,1);
nu = size(B,2);


% lambVec = zeros(nl,N);
% 
% for k=1:N
%     lambVec(:,k) = eig(Arho(nx*(k-1)+1:nx*k,:)+Brho(nx*(k-1)+1:nx*k,:)*K);
% end
% lambda0 = eig(A+B*K);

Fig =ieee_fig;
cmap = parula(N);
colororder(cmap);

taxiss = tiledlayout(3,2);

for k=1:N
    AA = Arho(nx*(k-1)+1:nx*k,:)+Brho(nx*(k-1)+1:nx*k,:)*K;
    [ReData,ImgData,~] = nyquist(ss(AA,Brho(nx*(k-1)+1:nx*k,:),C,[]));
    
    re_(:,1) = ReData(1,1,:);
    im_(:,1) = ImgData(1,1,:);

    re_(:,2) = ReData(1,2,:);
    im_(:,2) = ImgData(1,2,:);

    re_(:,3) = ReData(2,1,:);
    im_(:,3) = ImgData(2,1,:);

    re_(:,4) = ReData(2,2,:);
    im_(:,4) = ImgData(2,2,:);
    % wout_(:,1)= wout_nyq(1,1,:);
    
    for i=1:4
        ax = nexttile(taxiss,i);
        plot(ax,flip(re_(:,i)),flip(-im_(:,i)),'LineWidth',3,...
                            'Color',cmap(k,:)); 
        if k==1
            hold(ax,"on"); 
        end
        plot(ax,(re_(:,i)),(im_(:,i)),'LineWidth',3,...
                            'Color',cmap(k,:));
        % xlabel(ax,'Real-axis');
        % ylabel(ax,'Img-axis');
        switch i
            case 1
                title(ax,'$V^{r}_d$ to $i^{\Delta}_d$');
                if k==N
                    ylabel(ax,'Img-axis')
                end
            case 2
                title(ax,'$V^{r}_q$ to $i^{\Delta}_d$');
            case 3
                title(ax,'$V^{r}_d$ to $i^{\Delta}_q$');
                if k==N
                    ylabel(ax,'Img-axis')
                    xlabel(ax,'Real-axis')
                end
            case 4
                title(ax,'$V^{r}_q$ to $i^{\Delta}_q$');
                if k==N
                    xlabel(ax,'Real-axis')
                end
        end
        ax.XMinorGrid="on"; ax.YMinorGrid="on";
    end   

    re_ = [];
    im_ = [];
end
    % xlabel(taxiss,'Real-axis','Fontsize',16,'Interpreter','latex');
    % ylabel(taxiss,'Img-axis','Fontsize',16,'Interpreter','latex');

    c = colorbar;
    c.TickLabelInterpreter = 'latex';
    c.Label.Interpreter = 'latex';
    c.Label.String = '$\alpha$';
    c.Limits = [0 1];
    c.Ticks = ([0.1, 0.5, 1, 2, 3]-0.1)/2.9;
    c.TickLabels ={'0.1','0.5', '1','2', '3'};
    c.Layout.Tile = 'north';

    % c2 = colorbar(ax2,'Direction','normal');
    % c2.TickLabelInterpreter = 'latex';
    % c2.Label.Interpreter = 'latex';
    % c2.Label.String = '$(L_{m\Delta}, R_{m\Delta})$';
    % c2.Limits = [-0.9 2];
    % 
    % c3 = colorbar(ax3,'Direction','normal');
    % c3.TickLabelInterpreter = 'latex';
    % c3.Label.Interpreter = 'latex';
    % c3.Label.String = '$(L_{m\Delta}, R_{m\Delta})$';
    % c3.Limits = [-0.9 2];
    % 
    % c4 = colorbar(ax4,'Direction','normal');
    % c4.TickLabelInterpreter = 'latex';
    % c4.Label.Interpreter = 'latex';
    % c4.Label.String = '$(L_{m\Delta}, R_{m\Delta})$';
    % c4.Limits = [-0.9 2];

    % xlim(ax,[-1.1,0.1]);
    % ylim(ax,[-0.1,0.1]);
    
    % xlabel([ax,ax2,ax3,ax4],'Real-axis');
    % ylabel([ax,ax2,ax3,ax4],'Img-axis');
    % title(ax,'$V^{r}_d$ to $i^{\Delta}_d$');
    % title(ax2,'$V^{r}_d$ to $i^{\Delta}_q$');
    % title(ax3,'$V^{r}_q$ to $i^{\Delta}_d$');
    % title(ax4,'$V^{r}_q$ to $i^{\Delta}_q$');

    % ax.XMinorGrid="on"; ax.YMinorGrid="on";
    % ax2.XMinorGrid="on"; ax2.YMinorGrid="on";
    % ax3.XMinorGrid="on"; ax3.YMinorGrid="on";
    % ax4.XMinorGrid="on"; ax4.YMinorGrid="on";
end