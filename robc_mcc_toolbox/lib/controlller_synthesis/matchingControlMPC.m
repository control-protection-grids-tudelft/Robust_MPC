function [Q,R,P,J,diagnostic] = matchingControlMPC(K,A,B,N,vj,sigma,Pc,xij,type)
%  matchingControlMPC(K,A,B,N,vj,sigma,Pc,xij,type) generates 
% the matrices (Q,R,P) for tuning the MPC. This method follows the 
% solution proposed in [2,3].
% ---------------------------------------------------------------------- %
% Inputs: 
%  K    ->      [Ohm] computed state feedback gain determinng the closed
%                     loop performance. 
% A     ->      [matrix] state matrix
% B     ->      [matrix] input matrix
% C     ->      [matrix] output matrix 
% N     ->      [scalar] prediction horizon
% vj    ->      [matrix] contains the vector of the polytope X as in [2]
% xij   ->      [matrix] contains the vector of the polytope P as in [2]
% sigma ->      [scalar] hypervalue to ensure R>0, then sigma \in (0,1]
% Pc    ->      [matrix] imposed terminal set Pc
% type:         'classic'- computes optimal weight matrices as [1]
%               'relax'  - computes optimal weight matrices as [2]
%               'robust' - computes optimal weight matrices as [3] 
%               'nonquadratic' - computes optimal weight matrices as [2]
%                                   without optimal quadratic Lyapunov 
%                                   stability
% ---------------------------------------------------------------------- %
% Outputs:
% Q     ->      [matrix] state weigth matrix
% R     ->      [matrix] input weigth matrix
% P     ->      [matrix] terminal cost matrix 
% J     ->      [scalar] value function of J as [2]
% diagnostic -> [struct] returns the diagnostic from Yalmip solver
% ---------------------------------------------------------------------- %
% Authors: (i) Victor Daniel Reyes Dreke.
% ---------------------------------------------------------------------- %
% References:
% ---------------------------------------------------------------------- %
% [1]   Di Cairano, Stefano, and Alberto Bemporad. "Model predictive 
% control tuning by controller matching." IEEE Trans. on Aut. Cont. 55.1 
% (2009): 185-190.
% [2]   Dreke, Victor Daniel Reyes, and Mircea Lazar. "Practical deadbeat 
% mpc design via controller matching with applications in power 
% electronics." European Journal of Control 74 (2023): 100834.
% [3]   Dreke, Victor Daniel Reyes, Rahul Rane, and Aleksandra Lekić. 
% "Robust Tuning of Model Predictive Control for MMC-Based High-Voltage 
% Power Systems." arXiv preprint arXiv:2606.11948 (2026).
% ====================================================================== %

nx = size(B,1);
nu =size(B,2);

if isempty(Pc)==true
    pcFlag ='off';
else
    pcFlag ='on';
end

% Necessary condition for building Kcal
if N<size(A,2)
    warning(['Prediction horizon N must be larger or equal than n' ...
        '(system dimension). Hence, N is updated to 2n.'])
    N = nx*2;
end

% Kcal, i.e., [2, (16)].
Kcal = zeros(size(K,1)*N,size(K,2));

for i=1:N
    if i==1
        Kcal((i-1)*size(K,1)+1:i*size(K,1),:)=K;
    else
        Kcal((i-1)*size(K,1)+1:i*size(K,1),:)=K*(A+B*K)^(i-1);
    end
end

% Phi and Gamma Computation, i.e., compact prediction model as in [2, (9)]:
[ Phi, Gamma ] = ABN2PhiGamma(A,B,N);


% Decision variables definition:
Q_var = sdpvar(nx,nx);
P_var = sdpvar(nx,nx);
R_var = sdpvar(nu,nu);


% Definition of the block variable decision
Qcal = blkvar;
for i=1:N
    
    if i<N
        Qcal(i,i)= Q_var;
    elseif i==N
        switch pcFlag
            case 'off'
                Qcal(i,i) = P_var;
            case 'on'
                Qcal(i,i) = Pc;
            otherwise
                Qcal(i,i) = P_var;
        end
    end
end
Qcal = sdpvar(Qcal);

Rcal = blkvar;
for i=1:N  
    Rcal(i,i)= R_var;
end
Rcal = sdpvar(Rcal);


if string(type)=="relax"
    epsilon = sdpvar(1);
elseif string(type)=="robust"
    epsilon = sdpvar(1);
elseif string(type)=="nonquadratic"
    epsilon = sdpvar(1);
end

% LMI Defintion
% Constraints:

constraints = [] ;
% (1) Hard invariant constraints for the weigthing matrices
constraints = [constraints; Q_var>=0];  
constraints = [constraints; R_var>=sigma*eye(size(B,2))];
switch pcFlag
    case 'on'
        constraints = [constraints; P_var==Pc];
    case 'off'
        constraints = [constraints; P_var>=0];
    otherwise
        constraints = [constraints; P_var>=0];
end

% (2) Stability constraints and objective function
switch type
    case 'classic'

        objective_function = norm((Rcal+Gamma'*Qcal*Gamma)*Kcal ...
            +Gamma'*Qcal*Phi,2);

        constraints = [constraints; (A+B*K)'*P_var*(A+B*K)+...
            K'*R_var*K+Q_var-P_var<=0];

    case 'robust'
        objective_function = norm(1e2*epsilon,1);

        for i=1:size(vj,1)
            constraints = [constraints;...
                norm(((Rcal+Gamma'*Qcal*Gamma)*Kcal...
                +Gamma'*Qcal*Phi)*vj(i,:)',2)<=epsilon] ;
        end

        for i=1:size(xij,1)
            Aunc = A+reshape(xij(i,1:nx^2),[nx,nx]);
            Bunc = B+reshape(xij(i,nx^2+1:end),[nx,nu]);
            constraints = [constraints; (Aunc+Bunc*K)'*P_var*(Aunc+Bunc*K)+...
            K'*R_var*K+Q_var-P_var<=0];
        end        
    case 'relax'
        objective_function = norm(epsilon,1);

        for i=1:size(vj,1)
            constraints = [constraints;...
                norm(((Rcal+Gamma'*Qcal*Gamma)*Kcal...
                +Gamma'*Qcal*Phi)*vj(i,:)',2)<=epsilon] ;
        end

        constraints = [constraints; (A+B*K)'*P_var*(A+B*K)+...
            K'*R_var*K+Q_var-P_var<=0];

    case 'nonquadratic'
        objective_function = norm(epsilon,1);

        for i=1:size(vj,1)
            constraints = [constraints;...
                norm(((Rcal+Gamma'*Qcal*Gamma)*Kcal...
                +Gamma'*Qcal*Phi)*vj(i,:)',2)<=epsilon] ;
        end

        constraints = [constraints; (A+B*K)'*P_var*(A+B*K)-P_var<=0];
    otherwise
        warning(['Type needs to be char array. Available options are' ...
            '{classic, robust, relax, nonquadratic}.'])
        error('Please defien a valid type. See previous warning.')
end

%%
% Optimization
ops = sdpsettings('solver','mosek','verbose',0);
diagnostic = optimize(constraints,objective_function,ops);

Q = value(Q_var);
P = value(P_var);
R = value(R_var);
J = value(objective_function);


end