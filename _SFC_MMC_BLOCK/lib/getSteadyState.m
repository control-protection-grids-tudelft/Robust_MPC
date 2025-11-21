function [xss,uss,erorPredict]=getSteadyState(A,B,C,D,E,y,d,xss,method)
% getSteadyState(A,B,C,D,E,y,d,xss,method) computes the steady-state such
% that xss = Axss +Buss.
% ---------------------------------------------------------------------- %
if method=="state"
   c = -(A-eye(size(A,1)))*xss;
   uss = (B'*B)\(B'*c); % uss = pinv(B)*c;   
   % check accuracy 
   erorPredict = (xss-A*xss-B*uss);   
elseif method=="output"
    warning('to do') % to do
elseif method=="pertuberd"
    warning('to do') % to do
else
    warning('to do') % to do
end
end