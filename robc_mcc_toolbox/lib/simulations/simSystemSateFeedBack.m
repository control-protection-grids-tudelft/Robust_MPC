function  [x,u]= simSystemSateFeedBack(A,B,K,x0,tsim,x_ss,u_ss)
% simSystemSateFeedBack(A,B,K,x0,tsim) simulate the discrete an state-space
% representation in closed-loop with state feedback gain.
% simSystemSateFeedBack(A,B,K,x0,tsim,x_ss,u_ss)
% ---------------------------------------------------------------------- %
x = zeros(size(x0,1),tsim);
x = [x0, x];
u = zeros(size(B,2),tsim);

switch nargin
    case 5  %  Closed-loop stabilizing the origin
        for i=1:tsim
            u(:,i) = K*x(:,i); 
            x(:,i+1) =  (A*x(:,i)+B*u(:,i));
        end
    case 7  %  Closed-loop stabilizing equilibrium points      
        z = zeros(size(x0,1),tsim+1);
        method = "state";
        for i=1:tsim
            [~,uss,erorPredict] = getSteadyState(A,B,[],[],[],[],[],...
                                                    x_ss(:,i),method);
            % if norm(uss)>0
            %     pause
            % end
            norm(erorPredict)
            if norm(erorPredict)>1e-3
               pause
            end
            z(:,i) = x(:,i) - x_ss(:,i);
            u(:,i) = K*z(:,i) + uss; %u_ss(:,i); 
            x(:,i+1) =  (A*x(:,i)+B*u(:,i));
        end 
    otherwise
        error('Check your input data.')
end




end