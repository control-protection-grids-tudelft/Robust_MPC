function flag=checkSystem(A,B)
% checkSystem(A,B) checks controllability
% ------------------------------------
% Author: Victor Daniel Reyes Dreke
% ------------------------------------

if rank(ctrb(A,B)) == size(A,2)
    disp("Ok")
    flag =true;
else
    warning("Not controllable")
    flag =false;
end

end