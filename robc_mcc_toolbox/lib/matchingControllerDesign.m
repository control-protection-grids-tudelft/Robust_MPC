function [Q,R,P,J,diagnostic]=matchingControllerDesign(K,A,B,N,sigma,Pc)
% matchingControllerDesign(K,A,B,N,sigma) generates the matrices (Q,R,P)
% for tuning the MPC. This method follows the solution proposed in [1].
% ---------------------------------------------------------------------- %
% Authors: (i) Victor Daniel Reyes Dreke.
% ---------------------------------------------------------------------- %
% References:
% ---------------------------------------------------------------------- %
% [1]   Di Cairano, Stefano, and Alberto Bemporad. "Model predictive 
% control tuning by controller matching." IEEE Trans. on Aut. Cont. 55.1 
% (2009): 185-190.
% ====================================================================== %
if nargin==5
    Pc = [];
end

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
% sigma = 10^0; %sdpvar(1);

Qcal = blkvar;
for i=1:N
    
    if i<N
        Qcal(i,i)= Q_var;
    elseif i==N
        if isempty(Pc)==true
            Qcal(i,i) = P_var;
        else
            Qcal(i,i) = Pc;
        end
    end
end
Qcal = sdpvar(Qcal);

Rcal = blkvar;
for i=1:N  
    Rcal(i,i)= R_var;
end

Rcal = sdpvar(Rcal);


% constraints = [Q_var>=0; P_var>=0; R_var>=sigma*eye(size(B,2))];
if isempty(Pc)==true
    constraints = [Q_var>=0; P_var>=0; R_var>=sigma*eye(size(B,2)); (A+B*K)'*P_var*(A+B*K)+K'*R_var*K+Q_var-P_var<=0];
    % constraints = [Q_var==diag([0,0,1,1]); P_var>=0; R_var>=0; (A+B*K)'*P_var*(A+B*K)+K'*R_var*K+Q_var-P_var<=0];
else
    constraints = [P_var==Pc; Q_var>=0; R_var>=sigma*eye(size(B,2)); (A+B*K)'*Pc*(A+B*K)+K'*R_var*K+Q_var-Pc<=0];
end
objective_function = norm((Rcal+Gamma'*Qcal*Gamma)*Kcal +Gamma'*Qcal*Phi,2);

ops = sdpsettings('solver','mosek','verbose',0);
diagnostic = optimize(constraints,objective_function,ops);

Q =value(Q_var);
P = value(P_var);
R = value(R_var);
J =  value(objective_function);

end