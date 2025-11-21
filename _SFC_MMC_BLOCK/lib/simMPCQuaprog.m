function [xvec,uvec]= simMPCQuaprog(A,B,Q,P,R,N,x0,xbound,ubound,...
                                    x_ss,u_ss,tsim)
% simMPCQuaprog(A,B,Q,P,R,N,x0,xbound,ubound,x_ss,u_ss,tsim) computes the
% closed-loop response of the system in closed-loop with the MPC. In this
% case, the MPC optimization is solved with quadprog function from MATLAB.
% ---------------------------------------------------------------------- %
% Inputs:
% A      ->      [matrix] state matrix
% B      ->      [matrix] input matrix
% Q      ->      [matrix] state weigth matrix
% R      ->      [matrix] input weigth matrix
% P      ->      [matrix] terminal cost matrix 
% N      ->      [scalar] prediction horizon
% x0     ->      [nx1] initial state
% xbound ->      [nx2] state constraints as {xmin,xmax}
% ubound ->      [mx1] input constraints as {umin,umax}
% x_ss   ->      [nx1] steady-state states
% u_ss   ->      [mx1] steady-state inputs
% Tsim   ->      [scalar] simulation time
% ---------------------------------------------------------------------- %
% Outputs:
% xvec   ->      [n x Tsim] state vector  
% uvec   ->      [m x Tsim] input vector 
% ---------------------------------------------------------------------- %
% Authors: (i) Duo Xu and (ii) Victor Daniel Reyes Dreke.
% ---------------------------------------------------------------------- %
% References:
% ---------------------------------------------------------------------- %
% [1]   Lazar, Mircea. "Model predictive control of hybrid systems: 
% Stability and robustness." (2006).
% ====================================================================== %
xmax = xbound(:,2);
xmin = xbound(:,1);

umax = ubound(:,2);
umin = ubound(:,1);

[ Phi, Gamma ] = ABN2PhiGamma(A,B,N);
[ Psi, Omega ] = QRPN2PsiOmega( Q,R,P,N );

G = 2*(Psi+Gamma'*Omega*Gamma);
F = 2*Gamma'*Omega*Phi;

xvec = zeros(size(x0,1),tsim);
xvec = [x0, xvec];
uvec = zeros(size(B,2),tsim);

% Uncomment if you do not use Mosek
% options_qp =  optimoptions('quadprog','Display','off');
options_qp = [];
for k=1:tsim
    
    [ W, L, c] = getWLc( A, B, xmax-x_ss(:,k),...
                                xmin-x_ss(:,k),...
                                umax-u_ss(:,k),...
                                umin-u_ss(:,k),...
                                Gamma, Phi);
    x_k = xvec(:,k);
    z_k = x_k - x_ss(:,k);    
    try
        [u_qp,~,exitflag] = quadprog(G,F*z_k,L,c+W*z_k,...
                                        [],[],[],[],[],options_qp);
    catch
        warning('quadprog is solved using Mosek')
         [u_qp,~,exitflag] = quadprog(G,F*z_k,L,c+W*z_k,...
                                        [],[],[],[],[],options_qp);
    end
    
    if exitflag ~= 1
        warning('exitflag quadprog =%d\n', exitflag)
        if exitflag == -2
            warning('Optimization problem is infeasible. \n')
            u_qp = -inv(G)*F*z_k;
        end
    end
    
    uvec(:,k) = u_qp(1:size(B,2))+u_ss(:,k);
    xvec(:,k+1) = A*xvec(:,k) + B*uvec(:,k);
end

end