function [Au,bu]=getAUbu(N,umax,umin)
% =getAUbu(N,umax,umin)
nu = size(umin,1);
Nnu = N*nu;

% Building Ax 
Au =  zeros(2*Nnu,Nnu);
% Filling  Ax top half, i.e., xmin
for i=1:Nnu
    for j=1:Nnu
        if i==j
            Au(i,j) = -1;       % minimum constraints
            Au(Nnu+i,j) = 1;    % maximum constraints
        end
    end
end


% Building U_ (Umin) and _U (Umax)
Umin = zeros(Nnu,1);
Umax = zeros(Nnu,1);

% Filling  Umax and Umin
for i=1:N
    Umin((i-1)*nu+1:nu*i,1) = -umin;
    Umax((i-1)*nu+1:nu*i,1) = umax;
end

% Building b 
bu = zeros(2*Nnu,1);

% Filling b
for i=1:Nnu
    bu(i,1) = Umin(i,1);
    bu(Nnu+i,1) = Umax(i,1);
end

end