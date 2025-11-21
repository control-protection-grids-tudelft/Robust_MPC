function [K] = deadbeatControllerDesing(A,B)
% deadbeatControllerDesing(A,B) compute the deadbeat feedback gain 
% for the (A,B) realization based on the diferent formulation. The 
% resulting stable close-loop yields x(k+1) = (A+BK)x(k).  
% ---------------------------------------------------------------------- %
% Authors: (i) Victor Daniel Reyes Dreke,
% ---------------------------------------------------------------------- %
% References:
% ---------------------------------------------------------------------- %
% [1]   Ackermann, J. "Entwurf durch Polvorgabe/Design by pole placement." 
% at-Automatisierungstechnik 25.1-12 (1977): 173-179.
% [2]   Quevedo, Daniel E., Ricardo P. Aguilera, and Tobias Geyer.  
% "Predictive control in power electronics and drives: Basic concepts, 
% theory, and methods." Advanced and Intelligent Control in Power  
% Electronics and  Drives. Cham: Springer International Publishing, 2014. 
% 181-226.
% ====================================================================== %
m = size(B,2);
    if m ==1 % Ackerman Equation from [1]
        n = size(A,2);
        controllability_matrix =  ctrb(A,B);

        sTr = zeros(1,size(controllability_matrix,2));
        sTr(1,size(controllability_matrix,2))=1;
        
        if size(controllability_matrix,1) == size(controllability_matrix,2)
            sTr = sTr*(inv(controllability_matrix));
        else
            sTr = sTr*(inv(controllability_matrix'*controllability_matrix)*controllability_matrix');
        end

        K = sTr*A^n;
        K = -K;

        if (A+B*K)^(n+n) == zeros(size(A,2))
            disp("Ok")
        else
            warning("Nilpotent property missing")
        end

    elseif m==size(A,1) % Quevedo formulation from [2]
        K = -(inv(B))*A;
        
    elseif m<size(A,1)== true % Quevedo formulation from [2]
        K =  -(inv(B'*B)*B')*A;
    else
        K = deadbeat_state_feedback(A,B);
        
   end


end