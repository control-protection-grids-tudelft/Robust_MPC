function [Axu,b,Mx]=getAXbx(Gamma,Phi,N,xmax,xmin)
% getAXbx(Gamma,Phi,N,Xmax,Xmin,nu,nx)
nx = size(xmin,1);
Nnx = N*nx;

% Building Ax 
Ax =  zeros(2*Nnx,Nnx);
% Filling  Ax top half, i.e., xmin
for i=1:Nnx
    for j=1:Nnx
        if i==j
            Ax(i,j) = -1;       % minimum constraints
            Ax(Nnx+i,j) = 1;    % maximum constraints
        end
    end
end

% Building Axu
Axu =  Ax*Gamma;

% Building X_ (Xmin) and _X (Xmax)
Xmin = zeros(Nnx,1);
Xmax = zeros(Nnx,1);
% Filling  Xmax and Xmin
for i=1:N
    Xmin((i-1)*nx+1:nx*i,1) = -xmin;
    Xmax((i-1)*nx+1:nx*i,1) = xmax;
end
% Building b 
b = zeros(2*Nnx+nx,1);

% Filling b
for i=1:Nnx
    b(i,1) = Xmin(i,1);
    b(Nnx+i,1) = Xmax(i,1);
end

% Filling Mx 
Mx =[eye(2*Nnx),-Ax*Phi];

% Recalling bx = Mx*b;
end