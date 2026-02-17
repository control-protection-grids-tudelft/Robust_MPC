function [Adu,bdu]=getAdUbdu(N,umax,umin)
% getAiUbiu(N,umax,umin)
nu = size(umin,1);
Nnu = N*nu;

% Building Au 
Au =  zeros(2*Nnu,Nnu);
% Filling  Ax top half, i.e., xmin
for i=1:Nnu
    for j=1:Nnu
        if i<=j
            Au(i,j) = -1;       % minimum constraints
            Au(Nnu+i,j) = 1;    % maximum constraints
        end
    end
end

% Building Eu
Eu =  zeros(Nnu,Nnu);
for i=1:Nnu
    for j=1:Nnu
        if i==j
            Eu(i,j) = 1;        % u(k)
            if i>1
                Eu(i,j-1) = -1;     % -u(k-1)
            end
        end
    end
end

% Building Adu
Adu = Au*Eu;

% Building iU_ (iUmax) and _iU (iUmin)
dUmin = zeros(Nnu,1);
dUmax = zeros(Nnu,1);
% Filling  Xmax and Xmin
for i=1:N
    dUmin((i-1)*nu+1:nu*i,1) = -umin;
    dUmax((i-1)*nu+1:nu*i,1) = umax;
end
% Building biu 
bdu = zeros(2*Nnu,1);

% Note that bdu needs to be modify such as bdu(1) = bdu +upast

end