function checkSystem(A,B)
% checkSystem(A,B)
if rank(ctrb(A,B)) == size(A,2)
    disp("Ok")
else
    warning("Not controllable")
end

end