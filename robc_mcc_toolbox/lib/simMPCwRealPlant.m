function [xvec,uvec]= simMPCwRealPlant(A,B,Ap,Bp,Q,P,R,N,x0,xbound,ubound,...
                                    x_ss,u_ss,tsim,augHead,dvec,MPCType)
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

nx =  length(xmax);
nu =  length(umax);
u0 = umax*0;

if nargin==14
    augHead = false;
    dvec = zeros(nx,tsim+1);
end

[ Phi, Gamma ] = ABN2PhiGamma(A,B,N);
[ Psi, Omega ] = QRPN2PsiOmega(Q,R,P,N ); 
[Mcal,b] = getMb(umax*4.4,umin*4.4,N); warning('Line 51 is modified');

if augHead == false
    Mcal = Mcal*0;
end

if nargin==16
    MPCType = 'classic';
    G = 2*(Psi+Gamma'*Omega*Gamma);
    F = 2*Gamma'*Omega*Phi;
    PsiZ = Psi*0;
elseif nargin==17
    switch MPCType
        case 'classic'
            G = 2*(Psi+Gamma'*Omega*Gamma);
            F = 2*Gamma'*Omega*Phi;
            PsiZ = Psi*0;
        case 'input_rate'
            Z = zeros(N,N);
            for i=1:N
                for j=1:N
                    % Diagonal term
                    if j==i 
                        Z(i,j)=1;
                    end
                    % Off Diagonal term
                    if j==i-1
                       Z(i,j)=-1;
                   end
               end
            end
            PsiZ = kron(Z,eye(size(B,2)));
            G = 2*(PsiZ'* Psi* PsiZ+ Gamma'* Omega* Gamma);
            F = 2* Gamma'* Omega* Phi;
        otherwise % Default mode
            G = 2*(Psi+Gamma'*Omega*Gamma);
            F = 2*Gamma'*Omega*Phi;
            PsiZ = Psi*0;
    end    
end


% bpast = [ones(2*nu,1); zeros(2*nu*(N-1),1)]; 

xvec = zeros(size(x0,1),tsim);
xvec = [x0, xvec];
uvec = zeros(size(B,2),tsim+1);
upast = [u0;zeros((N-1)*nu,1)];

% Uncomment if you do not use Mosek
% options_qp =  optimoptions('quadprog','Display','off');
options_qp = [];
for k=1:tsim
    
    [ W, L, c] = getWLc( A, B, xmax-x_ss(:,k),...
                                xmin-x_ss(:,k),...
                                umax-u_ss(:,k),...
                                umin-u_ss(:,k),...
                                Gamma, Phi);
    M = [L;Mcal];
    x_k = xvec(:,k);
    z_k = x_k - x_ss(:,k); 
    bpast = [ones(nu,1).*u0; -ones(nu,1).*u0; zeros(2*nu*(N-1),1)]; 
    gamma = [c+W*z_k;b-bpast];

    if augHead == false
        gamma = [c+W*z_k;0*b-0*bpast];
    end
    try
        [u_qp,~,exitflag] = quadprog(G,F*z_k-0*PsiZ*upast,M,gamma,...
                                        [],[],[],[],[],options_qp);
        % [u_qp,~,exitflag] = quadprog(G,F*z_k,L,c+W*z_k,... Orignal
        %                                 [],[],[],[],[],options_qp);
    catch
        warning('quadprog is solved using Mosek')
         [u_qp,~,exitflag] = quadprog(G,F*z_k-0*PsiZ*upast,M,gamma,...
                                        [],[],[],[],[],options_qp);
    end
    
    if exitflag ~= 1
        warning('exitflag quadprog =%d\n', exitflag)
        if exitflag == -2
            warning('Optimization problem is infeasible. \n')
            u_qp = -inv(G)*(F*z_k+0*PsiZ*upast);
        else
            warning('Something weird happend. \n')
            u_qp = -inv(G)*(F*z_k+0*PsiZ*upast);
        end
    end
    
    uvec(:,k) = u_qp(1:size(B,2))+u_ss(:,k);
    xvec(:,k+1) = Ap*xvec(:,k) + Bp*uvec(:,k)+dvec(:,k);

    if augHead == true
        u0 = u0 + uvec(:,k);
    end
    upast = [uvec(:,k);zeros((N-1)*nu,1)];
    
end

end