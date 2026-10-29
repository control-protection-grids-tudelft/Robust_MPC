function [M,Mxr,Mur] = getOffSetDisturbanceParam(A,B,C,D)
%  getOffSetDisturbanceParam(A,B,C,D)

    M   = inv([A - eye(n), B; C, D]);                 % needs no zero at z = 1
    Mxr = M(1:n, n+1:end);      Mxd = -M(1:n, 1:n);   % Bd = I
    Mur = M(n+1:end, n+1:end);  Mud = -M(n+1:end, 1:n);
    
% Example online use
% res  = x - A*x_prev - B*u_prev;        % u_prev = input actually applied
% dhat = dhat + Ld*(res - dhat);
% xs   = Mxr*r + Mxd*dhat;
% us   = Mur*r + Mud*dhat;
% u    = min(max(us - K*(x - xs), umin), umax);
% x_prev = x;  u_prev = u;
end