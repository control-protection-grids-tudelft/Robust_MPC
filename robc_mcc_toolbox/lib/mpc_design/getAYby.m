function [Ayu,b,My]=getAYby(Gamma,Phi,C,N,xmax,xmin)
% getAXbx(Gamma,Phi,N,Xmax,Xmin,nu,nx)
ymax = C*xmax;
ymin = C*xmin;

ny = size(ymin,1);
nx = size(xmin,1);

Nnx = N*nx;
Nny = N*ny;

% Building Ck
Ck = zeros(Nny,Nnx);
for i=1:N
    Ck(ny*(i-1)+1:ny*i,nx*(i-1)+1:nx*i) = C;
end

% Building Ay 
Ay =  zeros(2*Nny,Nny);
% Filling  Ay top half, i.e., ymin
for i=1:Nny
    for j=1:Nny
        if i==j
            Ay(i,j) = -1;       % minimum constraints
            Ay(Nny+i,j) = 1;    % maximum constraints
        end
    end
end

% Building Ayu
Ayu =  Ay*Ck*Gamma;

% Building Y_(Ymin) and _Y (Ymax)
Ymin = zeros(Nny,1);
Ymax = zeros(Nny,1);
% Filling  Xmax and Xmin
for i=1:N
    Ymin((i-1)*ny+1:ny*i,1) = -ymin;
    Ymax((i-1)*ny+1:ny*i,1) = ymax;
end
% Building b 
b = zeros(2*Nny+nx,1);

% Filling b
for i=1:Nny
    b(i,1) = Ymin(i,1);
    b(Nny+i,1) = Ymax(i,1);
end

% Filling Mx 
My =[eye(2*Nny),-Ay*Ck*Phi];

% Recalling bx = Mx*b;
end