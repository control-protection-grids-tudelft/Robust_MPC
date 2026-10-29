function [M,b] = getMb(umax,umin,N)
 % ok
    nu = size(umax,1);
    Ei = [eye(nu);-eye(nu)];
    M = zeros(2*nu*N,nu*N);
    b = zeros(2*nu*N,1);

    for i=1:N

        b((i-1)*2*nu+1:(i*2*nu)) = [umax;-umin];

        Mi = zeros(2*nu,nu*N);

        for j=1:i
            Mi(:,(j-1)*nu+1:(j*nu)) = Ei;
        end

        M((i-1)*2*nu+1:(i*2*nu),:) = Mi;        
    end  
end