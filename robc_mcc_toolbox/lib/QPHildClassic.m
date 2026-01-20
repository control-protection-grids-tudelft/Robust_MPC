function eta = QPHildClassic(invOmega, f, M_cons, gamma, max_iter, tol)
    % QPhild - Solves Quadratic Programming using Hildreth's method
    %   Minimize:     0.5 * eta' * Omega * eta + f' * eta
    %   Subject to:   M_cons * eta <= gamma
    %
    % Inputs:
    %   Omega       - Hessian matrix (positive definite)
    %   f           - Gradient vector
    %   M_cons      - Constraint matrix (MxN)
    %   gamma       - Constraint bounds (Mx1)
    %
    % Output:
    %   eta     - Optimal solution vector
    % if nargin == 4
    %     max_iter = 38;
    %     tol = 1e-10;
    % end
    
    % Initial unconstrained solution
    % eta = -Omega \ f;
    eta = -invOmega*f;
    % Check if constraints are already satisfied
    violations = M_cons * eta - gamma;
    if all(violations <= 0)
        return;
    end
    
    % Precompute for iterative updates
    % P = M_cons * (Omega \ M_cons');
    % d = M_cons * (Omega \ f) + gamma;
    P = M_cons * (invOmega*M_cons');
    d = M_cons * (invOmega*f) + gamma;
    lambda = zeros(size(gamma));
    
    % Iterative projection (Hildreth’s method)
    for k = 1:max_iter
        lambda_prev = lambda;
        for i = 1:length(gamma)
            w = P(i,:) * lambda - P(i,i) * lambda(i);
            la = (-w - d(i)) / P(i,i);
            lambda(i) = max(0, la);
        end

        % Convergence check
        if norm(lambda - lambda_prev)^2 < tol
            break;
        end
    end
    
    % Compute final solution
    eta = eta - invOmega*M_cons' * lambda;
end
