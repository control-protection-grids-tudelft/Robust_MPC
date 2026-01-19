classdef MPCClass
    %MPCCLASS construct an MPC object to solve finite-time OCP problem
    %   MPCClass relates to the MPC object, where an fintie-time OCP
    %   problem is solved. The class properties include the ingridients to
    %   define an OCP problem for a discrete-time state-space realization 
    
    properties(SetAccess = private)
        % State-space realization
        A       % [nxn] state matrix
        B       % [nxm] input matrix
        C       % [pxn] output matrix
        D       
        E
        % Constraints
        Xbound          % [xmax, xmin]
        Ubound          % [umax, umin]    
        Pvec            % [nmx1]  scheduling variable vector
        % Compact prediction model
        Phi             % [Nnxn]    state dependent matrix
        Gamma           % [NnxNm]   input dependent matrix
        % Cost function ingridients
        N               % [scalar]  prediction horizon
        Q               % [nxn]
        R               % [mxm]
        P               % [nxn]
        Psi             % [NmxNm]
        Omega           % [NnxNn]
        % Terminal ingridients (Matching controller)
        XTerminal       % Yalmip set (pending)
        K               % Matching controller (Terminal controller)
        % QP matrices
        G               % [NmxNm] quadratic cost matrix 
        F               % [Nmxn]  affine cost matrix
        % Constraint matrices
        L               % [] matrix from 
        W               % [] matrix
        c               % [] vector 
        M               % [] matrix
        b               % [] vector
        % MPC properties
        methodT         % [string] Tuning method: 'classic', 'const', 'rob'
        methodC         % [string] Constraint method: 'sti', 'stdi','in'
        methodO         % [string] Optimization method        
    end
    
    methods
        function obj = MPCClass(A,B,C,Pvec,Xbound,Ubound,N,Q,R)
            %MPCCLASS Construct an instance of this class
            %  MPCClass main properties are the state-space realization
            %  matrices (A,B,C), bounds of the constraint and inputs  
            %  (Xbound,Ubound) and  the prediction horizon (N).
            obj.A = A;
            obj.B = B;
            obj.C = C;
            obj.Xbound = Xbound;   
            obj.Ubound = Ubound;
            obj.Pvec = Pvec;
            obj.N = N; 
            obj.Q = Q;
            obj.R = R;
        end
        
        function obj = createOCP(obj,sigma,methodT,methodC,methodO)
            %createOCP() construct the OCP problem
            %   Detailed explanation goes here
            obj.methodT = methodT;
            obj.methodC = methodC;
            obj.methodO = methodO;

            [obj.Phi,obj.Gamma]= ABN2PhiGamma(obj.A,obj.B,obj.N);
            
            % Matching controller construction
            switch methodT
                case 'dlqr'
                    [obj.K, obj.P,~] = dlqr(obj.A,obj.B,obj.Q,obj.R,[]);                    
                otherwise
                    [obj.K,~]=getRobustK(obj.A,obj.B,[],obj.Pvec,...
                                                            obj.Xbound,...
                                                            obj.Ubound,...
                                                            methodT);
                    [obj.Q,obj.R,obj.P,~,~]=matchingControllerDesign(obj.K,...
                                            obj.A,obj.B,obj.N,sigma);
            end

            [obj.Psi,obj.Omega] = QRPN2PsiOmega(obj.Q,obj.R,obj.P,obj.N);

            obj.G = 2*(obj.Psi+obj.Gamma'*obj.Omega*obj.Gamma);
            obj.F = 2*obj.Gamma'*obj.Omega*obj.Phi;

            [obj.W,obj.L,obj.c] = getWLc(obj.A,obj.B,...
                                        obj.Xbound(:,1),obj.Xbound(:,2),...
                                        obj.Ubound(:,1),obj.Ubound(:,2),...
                                        obj.Gamma, obj.Phi);
            [obj.M, obj.b] = getMb(obj.Ubound(:,1),obj.Ubound(:,2),...
                                    obj.N);
        end

        function u_qp = solveOneQP(obj,x0,u0)
            % Solve a QP problem
            invG = inv(obj.G);
            nu = length(u0);
            switch obj.methodC
                case 'sti'
                    MConst = obj.L;
                    bConst = obj.c+obj.W*x0;
                case 'stdi'
                    bpast = [ones(nu,1).*u0; ...
                            -ones(nu,1).*u0;...
                             zeros(2*nu*(obj.N-1),1)];
                    MConst = [obj.L; obj.M];
                    bConst = [obj.c+obj.W*x0;obj.b-bpast];
                case 'in'
                    MConst = obj.M;
                    bConst = obj.b;
            end
            switch obj.methodO
                case 'Hildreth'
                    max_iter = 38;
                    tol = 1e-6;
                    u_qp = QPHildClassic(invG, obj.F*x0, ...
                        MConst,bConst,max_iter, tol);
                case 'Quadprog'
                    options_qp = [];
                    [u_qp,~,exitflag] = quadprog(obj.G,obj.F*x0,...
                                                    MConst,bConst,...
                                                    [],[],[],[],[],...
                                                    options_qp);
                    if exitflag ~= 1
                        warning('exitflag quadprog =%d\n', exitflag)
                        if exitflag == -2
                            warning('Optimization problem is infeasible.\n')
                            u_qp = -invG*obj.F*x0;
                        else
                            warning('Something weird happend. \n')
                            u_qp = -invG*obj.F*x0;
                        end
                    end
            end
        end
    end
end

