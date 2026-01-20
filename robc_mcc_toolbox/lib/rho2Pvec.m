function Au = rho2Pvec(pvec)
    neq = 2*length(pvec);
    if isscalar(pvec)==true
        % Default uncertainty model,where only diagonal terms are uncertains
    
        vvec = kron([pvec, 0, 0, pvec]', [-1,1]); % Provisional 
        Au = [vvec;0*vvec];
        % Note: vvec = kron([pvec, 0, 0, pvec]', [-1,1]) - considers only an
        % uncertain variable. The [-1,1] needs to be replaced if we want to
        % include higher dimension (30-10-2025).
        % Fixed problem. 
    else    
        X = 0:2^(neq/2)-1;
        Y = int2bit(X,neq/2);
        vvec = 2*(Y - 0.5*ones(size(Y)));
        if size(pvec,2) > 1
            pvec = reshape(pvec,size(pvec,1)*size(pvec,2),1);
        end
        vvec = pvec.*vvec;
        [Au]=uniqueColumns(vvec);
    end
end