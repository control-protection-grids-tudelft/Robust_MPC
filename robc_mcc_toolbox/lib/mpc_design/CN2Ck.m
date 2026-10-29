function Ck = CN2Ck(C,N)
% Ck = CN2Ck(C,N) function to compound C matrices.
% ------------------------------------
% Author: Victor Daniel Reyes Dreke
% ------------------------------------

ny = size(C,1);
nx = size(C,2);

Nnx = N*nx;
Nny = N*ny;

Ck = zeros(Nny,Nnx);
for i=1:N
    Ck(ny*(i-1)+1:ny*i,nx*(i-1)+1:nx*i) = C;
end

end