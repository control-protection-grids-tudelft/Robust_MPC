function [Fig_full,XVec,UVec] = plotUncertaintSystem(A,B,K,Pvec,...
    x0,Tsim,Ts,w,xss,umax)
% plotUncertaintSystem(A,B,K,Pvec)


nx = size(A,1);
nu = size(B,2);
na = size(A,1)*size(A,2);
nb = size(B,1)*size(B,2);

if nargin==7
    w = zeros(nx,Tsim+1);
    xss = zeros(nx,Tsim+1);
    umax = 1;
elseif nargin==10
    if isempty(w) == true
         w = zeros(nx,Tsim+1);
    end
    if isempty(xss) == true
        xss = zeros(nx,Tsim+1);
    end
    if isempty(umax) == true
        umax = 1;
    end
end

if isscalar(Pvec)==true
    np = Pvec;
else
    np = size(Pvec,2);
end

ny_p = max([nx, nu]);

% State and input vector
xvec = [x0, zeros(size(x0,1),Tsim)];
uvec = zeros(size(B,2),Tsim+1);
uss = zeros(size(B,2),Tsim+1);
% cmap = hsv(NModel); 

% Compound
XVec = zeros(Tsim+1,(np)*nx);
UVec = zeros(Tsim+1,(np)*nu);

for i=1:np
    if isscalar(Pvec)==true
        PAvec = rand(nx,nx)-0.5*ones(nx,nx);
        PBvec = rand(nx,nu)-0.5*ones(nx,nu);
    else
        PAvec = reshape(Pvec(1:na,i),size(A))';
        PBvec = reshape(Pvec(na+1:na+nb,i),size(B,2),size(B,1))';
    end
    ATrue = (A+PAvec);
    BTrue = (B+0*PBvec);
   for k=1:Tsim
            uss(:,k) = (BTrue\(eye(nx)-ATrue))*xss(:,k);
            uvec(:,k) = min(umax, max(-umax, K*(xvec(:,k)-xss(:,k))+uss(:,k)));
            xvec(:,k+1)=(A+PAvec)*xvec(:,k)+(B+PBvec)*uvec(:,k)+w(:,k); 
   end    
   uss(:,Tsim+1) = (BTrue\(eye(nx)-ATrue))*xss(:,Tsim+1);
   uvec(:,Tsim+1) =  K*(xvec(:,Tsim+1)-xss(:,Tsim+1))+uss(:,Tsim+1);

    XVec(:,(i-1)*nx+1:i*nx) = xvec';
    UVec(:,(i-1)*nu+1:i*nu) = uvec';
end

cmap = cool(np);
numColum = 2;


Fig_full = ieee_fig();

xHeadLabel = {'d','q','z'};
for i=2:ny_p+1
    for j=0:1       
        if j==0
            ax = subplot(ny_p+1,numColum,numColum*(i-1)+1+j);      % poles map
            colororder(cmap)
            plot(ax,Ts*(0:Tsim)',XVec(:,i+1),'LineWidth',3,'Color','g')  
            hold(ax,"on")
            plot(ax,Ts*(0:Tsim)',xss(i-1,:)','--k','LineWidth',2.5)  
            plot(ax,Ts*(0:Tsim)',XVec(:,i+1:nx:np*nx),'LineWidth',2,'Color',cmap(np,:))  
            plot(ax,Ts*(0:Tsim)',XVec(:,i+1),'LineWidth',3,'Color','g') 
            plot(ax,Ts*(0:Tsim)',xss(i-1,:)','--k','LineWidth',2.5) 
            ylabel(ax,['$x_','{',num2str(i-1),'}$']) 
            ylabel(ax,['$i^{\Delta}_','{',xHeadLabel{i-1},'}$[p.u.]']) 
            xlabel(ax,'Time [s]')
            xlim(ax,'tight')
            ylim(ax,'tight')
            ax.XMinorGrid = 'on';  ax.YMinorGrid = 'on';
            legend(ax,{'nom','ref','unc'},'Location','southeast')
        elseif j==1
            ax = subplot(ny_p+1,numColum,numColum*(i-1)+1+j);      % poles map
            colororder(cmap)
            plot(ax,Ts*(0:Tsim)',UVec(:,i+1),'LineWidth',3,'Color','g')
            hold(ax,"on")
            plot(ax,Ts*(0:Tsim)',UVec(:,i+1:nu:np*nu),'LineWidth',2,'Color',cmap(np,:))
            plot(ax,Ts*(0:Tsim)',UVec(:,i+1),'LineWidth',3,'Color','g')
            ylabel(ax,['$u_','{',num2str(i-1),'}$'])
            ylabel(ax,['$u^{\Delta}_','{',xHeadLabel{i-1},'}$[p.u.]']) 
            xlabel(ax,'Time [s]')
            xlim(ax,'tight')
            ylim(ax,'tight')            
            ax.XMinorGrid = 'on';  ax.YMinorGrid = 'on';
            legend(ax,{'nom','unc'},'Location','northeast')
        else
        end
    end
end

% plot(XVec)
% ax = subplot(ny_p+1,2,1:2);      % poles map
% ax = subplot(ny_p+1,3,4:5);     % states i_d
% ax = subplot(3,3,7:8); % states i_q


end