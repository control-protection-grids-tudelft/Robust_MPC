function [Q,R,P,J,diagnostic]=matchingControllerDesignRelax(K,A,B,N,vj,sigma)
% matchingControllerDesignRelax(K,A,B,N,vector_matrix,sigma) generates 
% the matrices (Q,R,P) for tuning the MPC. This method follows the 
% solution proposed in [2].
% ---------------------------------------------------------------------- %
% Inputs: 
%  K    ->      [Ohm] computed state feedback gain determinng the closed
%                     loop performance. 
% A     ->      [matrix] state matrix
% B     ->      [matrix] input matrix
% C     ->      [matrix] output matrix 
% N     ->      [scalar] prediction horizon
% vj    ->      [matrix] contains the vector of the polytope P as in [2]
% sigma ->      [scalar] hypervalue to ensure R>0, then sigma \in (0,1]
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
% ====================================================================== %
if N<size(A,2)
    warning('Choosing N less than n. Hence, N will be changed to N')
    N= size(A,2)*2;
end

Kcal = zeros(size(K,1)*N,size(K,2));

for i=1:N
    if i==1
        Kcal((i-1)*size(K,1)+1:i*size(K,1),:)=K;
    else
        Kcal((i-1)*size(K,1)+1:i*size(K,1),:)=K*(A+B*K)^(i-1);
    end
end

[ Phi, Gamma ] = ABN2PhiGamma(A,B,N);

Q_var = sdpvar(size(A,2),size(A,2));
P_var = sdpvar(size(A,2),size(A,2));
R_var = sdpvar(size(B,2),size(B,2));

epsilon = sdpvar(1);
Qcal = blkvar;

for i=1:N
    
    if i<N
        Qcal(i,i)= Q_var;
    elseif i==N
        Qcal(i,i) = P_var;
    end
end

Qcal = sdpvar(Qcal);
Rcal = blkvar;

for i=1:N  
    Rcal(i,i)= R_var;
end

Rcal = sdpvar(Rcal);


constraints = [] ;
for i=1:size(vj,1)
    constraints = [constraints; norm(((Rcal+Gamma'*Qcal*Gamma)*Kcal +Gamma'*Qcal*Phi)*vj(i,:)',2)<=epsilon] ;
end
constraints_2 = [Q_var>=0; P_var>=0; R_var>=sigma*eye(size(B,2)); (A+B*K)'*P_var*(A+B*K)+K'*R_var*K+Q_var-P_var<=0];

ops = sdpsettings('solver','mosek','verbose',0);
% diagnostic = optimize(constraints,objective_function,ops);
diagnostic = optimize([constraints;constraints_2;epsilon>=0],epsilon, ops);

Q =value(Q_var);
P = value(P_var);
R = value(R_var);
J =  value(epsilon);

end