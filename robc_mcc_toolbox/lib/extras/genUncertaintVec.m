function Rho = genUncertaintVec(rmax,rmin)
% Rho = genUncertaintVec(rmax,rmin) to compute a set of vertices of
% uncertaint set, i.e., a polytopic set.
% -----------------------------------
% Inputs:   - rmax [n x 1] vector of maximum values where n is the number 
%                           of variables.
%           - rmin [n x 1] vector of minimum values where n is the number 
%                           of variables.
% Outputs:  - Rho [2^n x n] matrix with all the vertices
% ------------------------------------
% Author: Victor Daniel Reyes Dreke
% ------------------------------------

n = size(rmax,1);

Rho = zeros(2^n,n);

for j=1:2^n
    binStr = dec2bin(j-1,n);
    for i=1:n
        switch binStr(i)
            case '1'
                Rho(j,i)= rmax(i,1);
            case '0'
                Rho(j,i)= rmin(i,1);
            otherwise
                error('wrong binary code')
        end
        
    end
end

end