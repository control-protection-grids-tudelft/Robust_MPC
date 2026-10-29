function [Aiu,biu]=getAiUbiu(N,umax,umin)
% getAiUbiu(N,umax,umin)
nu = size(umin,1);
Nnu = N*nu;

% Building Ax 
Aiu =  zeros(2*Nnu,Nnu);
% Filling  Ax top half, i.e., xmin
for i=1:Nnu
    for j=1:Nnu
        if j<=i
            Aiu(i,j) = -1;       % minimum constraints
            Aiu(Nnu+i,j) = 1;    % maximum constraints
        end
    end
end



% Building iU_ (iUmax) and _iU (iUmin)
iUmin = zeros(Nnu,1);
iUmax = zeros(Nnu,1);
% Filling  Xmax and Xmin
for i=1:N
    iUmin((i-1)*nu+1:nu*i,1) = -umin;
    iUmax((i-1)*nu+1:nu*i,1) = umax;
end
% Building biu 
biu = zeros(2*Nnu,1);

% Note biu needs to be modify such as (2.99) Wang, Model Predicitive
% Control book.

end