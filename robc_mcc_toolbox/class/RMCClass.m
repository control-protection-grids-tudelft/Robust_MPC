classdef RMCClass
    %RMCCLASS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % State-space realization
        A               % [nxn] state matrix
        B               % [nxm] input matrix
        C               % [pxn] output matrix
        D       
        E
        % Dimensions 
        nx
        nu
        ny
        nw
        % Compound systems
        ABsys           % [cell] N x {A,B} cell array containing (A,B)
        % Constraints
        Xbound          % [xmax, xmin]
        Ubound          % [umax, umin]    
        % Static gains
        K               % [mxn] static feedback controller
        P               % [nxn] domain of attraction (DOA)
        % Methods
        methodControl   % [string]
    end
    
    methods
        function obj = RMCClass(A,B,C,D,E,Xbound,Ubound)
            % RMCCLASS Construct an instance of this class
            % RMCClass main properties are the nominal state-space 
            % realization matrices (A,B,C,D,E), bounds of the constraint   
            % and inputs (Xbound,Ubound) and  the prediction horizon (N).

            obj.ABsys = [];
            
            nx = size(A,1);
            nu = size(B,2);

            if isempty(C)==true
                C = eye(nx);
            end

            ny = size(C,1);
            if isempty(D)==true
                D = zeros(ny,nu);            
            end

            if isempty(E)==true
                E = zeros(nx,1); 
            end

            nw = size(E,2);
            
            obj.A = A;
            obj.B = B;
            obj.C = C;
            obj.D = D;
            obj.E = E;
            obj.Xbound = Xbound;   
            obj.Ubound = Ubound;
            obj.nu = nu;
            obj.nx = nx;
            obj.ny = ny;
            obj.nw = nw;
        end
        
        function [ABsys,obj] = getCompSystem(obj,PAvec,PBvec)
            % getCompSystem(PAvec,PBvec) 
            % functions 
            npa = size(PAvec,2);
            npb = size(PBvec,2);
            neq = max(npa,npb);

            if size(PAvec,1)<size(obj.A,1)*size(obj.A,2)
                warning('There is a dimension error dim(PAvec) >=nx^2.')
            end 

            if size(PBvec,1)<size(obj.B,1)*size(obj.B,2)
                warning('There is a dimension error dim(PAvec) >=nx x nu.')
            end 
            
            ABsys = cell(neq+1,2);
            ABsys{1,1} = obj.A; ABsys{1,2} = obj.B;
            for i=1:neq
                if i<=npa
                    ABsys{i+1,1} = obj.A + ...
                                reshape(PAvec(:,i),obj.nx,obj.nx)';
                else
                     ABsys{i+1,1} = obj.A + zeros(obj.nx,obj.nx);
                end

                if i<=npb
                    ABsys{i+1,2} = obj.B + ...
                                reshape(PBvec(:,i),obj.nx,obj.nu)';
                else
                    ABsys{i+1,2} = obj.B + zeros(obj.nx,obj.nu);
                end

            end

            obj.ABsys= ABsys;
        end
        
        function [K,P,diagnostics,obj] = synthClassic(obj,solverHead,tolVal)
            % synthClassic() function synthesizes a static feedback
            % gain that ensures quadratic stability under nominal dynamics.
            if nargin==1
                solverHead ='mosek';
                tolVal = 1e-6;
            elseif nargin==2
                if isempty(tolVal)==true
                    tolVal = 1e-6;
                end
            else
                if isempty(solverHead)==true
                    solverHead ='mosek';
                end
            end
            

            % sdpvar varaibles definition, i.e., optimization variables. 
            Z_i = sdpvar(obj.nx,obj.nx);            % inv(Z_i) =  P_i, due to Schur complement
            Y_i = sdpvar(obj.nu,obj.nx,'full');     % Y_i defines K as K = Y_i*P_i
            gamma_i = sdpvar(1,1);                  % Scalar factor 

            % LMI Definition
            L1 = [];
            M = [Z_i, (obj.A*Z_i+obj.B*Y_i)';(obj.A*Z_i+obj.B*Y_i), Z_i]; 
            L1 = [L1, M >= tolVal]; 
            L8 = gamma_i == 1;

            constraints = [L1, L8]; % Here good

            options = sdpsettings('solver',solverHead,'verbose',0);
            
            diagnostics = optimize(constraints,[],options);

            % Solution
            P = value(Z_i)^(-1);
            K = value(Y_i)*(value(Z_i)^(-1));
            obj.K = K;
            obj.P = P; 
            obj.methodControl = 'classic_quadratic';

        end
        function [K,P,diagnostics,obj] = synthClassicRobust(obj,PAvec,PBvec,solverHead,tolVal)
            % synthClassicRobust() 
            if nargin==1
                solverHead ='mosek';
                tolVal = 1e-6;
                if isempty(obj.ABsys) == true
                    error('The uncertainty vector are missing. PAvec and PBvec are missing.')
                end
            elseif nargin==3
                [~,obj] = getCompSystem(obj,PAvec,PBvec);
                tolVal = 1e-6;
                solverHead ='mosek';
            elseif nargin==4
                if isempty(PAvec)==false & isempty(PBvec)==false
                    [~,obj] = getCompSystem(obj,PAvec,PBvec);
                end
                if isempty(solverHead)==true
                    solverHead ='mosek';
                end
                tolVal = 1e-6;
            elseif nargin==5
                if isempty(solverHead)==true
                    solverHead ='mosek';
                end
                if isempty(tolVal)==true
                    tolVal = 1e-6;
                end
            end

            % sdpvar varaibles definition, i.e., optimization variables. 
            Z_i = sdpvar(obj.nx,obj.nx);            % inv(Z_i) =  P_i, due to Schur complement
            Y_i = sdpvar(obj.nu,obj.nx,'full');     % Y_i defines K as K = Y_i*P_i
            gamma_i = sdpvar(1,1); 

            % Vector definition
            vj =  size(obj.ABsys,1);

            % LMI Definition
            L1 = [];
            for i=1:vj
                M = [Z_i, (obj.ABsys{i,1}*Z_i+obj.ABsys{i,2}*Y_i)';
                      (obj.ABsys{i,1}*Z_i+obj.ABsys{i,2}*Y_i), Z_i];
                    
                L1 = [L1, M >= tolVal];    
            end  
            L8 = gamma_i == 1;

            constraints = [L1, L8]; % Here good

            options = sdpsettings('solver',solverHead,'verbose',0);
            
            diagnostics = optimize(constraints,[],options);

            % Solution
            P = value(Z_i)^(-1);
            K = value(Y_i)*(value(Z_i)^(-1));
            obj.K = K;
            obj.P = P; 
            obj.methodControl = 'classic_robust';

        end
        function [K,P,diagnostics,obj] = synthConstraintRobust(obj,PAvec,PBvec,solverHead,tolVal)
            % synthClassicRobust()
             % synthClassicRobust() 
            if isempty(obj.Xbound)==true || isempty(obj.Ubound)==true
                warnin('Object properties: Xbound and Ubound poorly defined. Using default constraints.')
                xmax = ones(obj.nx,1);
                xmin = -ones(obj.nx,1);
                umax = ones(obj.nu,1);
                umin = -ones(obj.nu,1);
            else
                xmax = obj.Xbound(:,2);
                xmin = obj.Xbound(:,1);
                umax = obj.Ubound(:,2);
                umin = obj.Ubound(:,1);
            end
            
            if nargin==1
                solverHead ='mosek';
                tolVal = 1e-6;
                if isempty(obj.ABsys) == true
                    error('The uncertainty vector are missing. PAvec and PBvec are missing.')
                end
            elseif nargin==3
                [~,obj] = getCompSystem(obj,PAvec,PBvec);
                tolVal = 1e-6;
                solverHead ='mosek';
            elseif nargin==4
                if isempty(PAvec)==false & isempty(PBvec)==false
                    [~,obj] = getCompSystem(obj,PAvec,PBvec);
                end
                if isempty(solverHead)==true
                    solverHead ='mosek';
                end
                tolVal = 1e-6;
            elseif nargin==5
                if isempty(solverHead)==true
                    solverHead ='mosek';
                end
                if isempty(tolVal)==true
                    tolVal = 1e-6;
                end
            end

            % sdpvar varaibles definition, i.e., optimization variables. 
            Z_i = sdpvar(obj.nx,obj.nx);            % inv(Z_i) =  P_i, due to Schur complement
            Y_i = sdpvar(obj.nu,obj.nx,'full');     % Y_i defines K as K = Y_i*P_i
            gamma_i = sdpvar(1,1); 

            % Vector definition
            vj =  size(obj.ABsys,1);

            % LMI Definition
            L1 = [];
            for i=1:vj
                M = [Z_i, (obj.ABsys{i,1}*Z_i+obj.ABsys{i,2}*Y_i)';
                      (obj.ABsys{i,1}*Z_i+obj.ABsys{i,2}*Y_i), Z_i];
                    
                L1 = [L1, M >= tolVal];    
            end  
            L8 = gamma_i == 1;

            nxConst = length(xmax);
            nuConst = length(umax);
                
            gij_max = diag(1./xmax);
            gij_min = diag(1./xmin);
                
            hij_max = diag(1./umax);
            hij_min = diag(1./umin);
            
            
            l3Temp = [];
            l4Temp = [];
            
            for k=1:nxConst
                tempMax = [Z_i, Z_i*gij_max(k,:)'; (Z_i*gij_max(k,:)')',...
                    gamma_i] >= 0;

                tempMin = [Z_i, Z_i*gij_min(k,:)'; (Z_i*gij_min(k,:)')',...
                    gamma_i] >= 0;

                l3Temp = [l3Temp, tempMax, tempMin];
            end
            
            for k=1:nuConst
                tempMax = [Z_i, (Y_i)'*hij_max(k,:)';...
                    ((Y_i)'*hij_max(k,:)')', gamma_i] >= 0;

                tempMin = [Z_i, (Y_i)'*hij_min(k,:)';...
                    ((Y_i)'*hij_min(k,:)')', gamma_i] >= 0;

                l4Temp = [l4Temp, tempMax, tempMin];
            end
            
            L3 = l3Temp;
            L4 = l4Temp; 

            constraints = [L1, L3, L4, L8]; % Here good

            options = sdpsettings('solver',solverHead,'verbose',0);
            
            diagnostics = optimize(constraints, -geomean(Z_i),options);

            % Solution
            P = value(Z_i)^(-1);
            K = value(Y_i)*(value(Z_i)^(-1));
            obj.K = K;
            obj.P = P; 
            obj.methodControl = 'constraint_robust';

        end

        function [K,P,miu] = synthDKIter()
        end

        function [K,gamma,obj] = synthHInfty(obj,Ac,Bc,Cc,Dc,Ec,...
                                                 Fe,Fd,Fu,Fr,...
                                                 Rbase,Dbase,Ubase,Ebase)
            % synthHInfty()
            % Note the plant should be normalized. 
            % Note Fe,Fd,Fu,Fr are structures that describe weighing
            % functions

            % Default values
            if nargin == 6
                we = 1; ke = 1;
                wd = 1; kd = 1; 
                wu = [1, 1]; ku = 1;  
                wr = 1; kr = 1;

                Fe = tf(ke*we,[1 we]);
                Fd = tf(kd*wd,[1 wd]);
                Fu = tf(ku*[1 2*wu(1) wu(1)^2],[1 2*wu(2) wu(2)^2]);
                Fr = tf(kr*wr,[1 wr]);

                Rbase =  tf(1,1);
                Dbase =  tf(1,1);
                Ubase =  tf(1,1);
                Ebase =  tf(1,1);
            end 
            
            % Following methodology from 
            Fe.InputName = 'e'; Fe.OutputName = 'tild_e';
            Fd.InputName = 'tild_d'; Fd.OutputName = 'd';
            Fu.InputName = 'u'; Fu.OutputName = 'tild_u';
            Fr.InputName = 'tild_r'; Fr.OutputName = 'r';
            
            Rbase.InputName = 'r'; Rbase.OutputName = 'R';
            Dbase.InputName = 'd'; Dbase.OutputName = 'D';
            Ubase.InputName = 'u'; Ubase.OutputName = 'U';
            Ebase.InputName = 'E'; Ebase.OutputName = 'e';
            
            Gnom = ss(Ac,[Bc Ec],Cc,Dc); 

            Gnom.InputName = {'D','U'};      % DU is the compound vector of U 
            % and D, where U are the system inputs and D are the system
            % disturbances. 
            Gnom.OutputName = 'Y';

            SUME = sumblk('E = R - Y', obj.ny);

            Gw = connect(Gnom,Dbase,Ubase,Ebase,Rbase,Fe,Fr,Fd,Fu,SUME,{'tild_r',...
                'tild_d','u'}, {'tild_e','tild_u','e'});

            [K,~,gamma] = hinfsyn(Gw,obj.ny,obj.nu);

        end

        function [K,P,Pu,diagnostics,obj]=synthOptRobust(obj,Ac,Bc,E1,E2,D1,Q,R,x0)
            % cite [Ayari, 2017] Multivariable Optimal Robust Control 
            % Strategy for MMC Converter

            tolVal = 1e-8;
            solverHead ='mosek';

            iQ = inv(Q);
            iR = inv(R);
            ne = size(E1,1) ;
            zc_xu = zeros(obj.nx,obj.nu);
            % zc_xx = zeros(obj.nx,obj.nx);
            zc_xe = zeros(obj.nx,ne);
            zc_ex = zeros(ne,obj.nx);
            zc_eu = zeros(ne,obj.nu);
            % zc_ee = zeros(ne,ne);
            zc_ux = zeros(obj.nu,obj.nx);
            zc_ue = zeros(obj.nu,ne);
            ic_ee = eye(ne);

            % Decision variable definitions
            X = sdpvar(obj.nx,obj.nx);
            Z = sdpvar(obj.nu,obj.nu);
            Y = sdpvar(obj.nu,obj.nx,'full');
            alph = sdpvar(1,1);
            varep = sdpvar(1,1);

            % LMI/Constraint definition
            % Positive alpha
            L1 = [];  
            L1 =[L1, alph>=tolVal];
            % Positive varepsilon
            L2 = [];  
            L2 =[L2, varep>=tolVal];
            % Level set for nominal initial conditions conditions 
            L3 = []; 
            M3 = [1, x0'; x0, X];
            L3 =[L3, M3>=tolVal];
            % Level set for input constraints 
            L4 = []; 
            M4 = [Z, Y'; Y, X];
            L4 =[L4, M4>=tolVal];
            % Input constraints
            umax = max(max(abs(obj.Ubound)))^2;
            L5 = [];
            for i=1:obj.nu
                L5 = [L5, Z(i,i)<=umax];
            end
            % Main LMI
            L6 = [];
            M6 = [(Ac*X+Bc*Y+(Ac*X+Bc*Y)'+varep*(D1*D1')), X, Y', (E1*X+E2*Y)';
                   X, -alph*iQ, zc_xu, zc_xe; 
                   Y, zc_ux, -alph*iR, zc_ue;
                   (E1*X+E2*Y), zc_ex, zc_eu,-varep*ic_ee];
            L6 =[L6, M6<=-tolVal];


            constraints = [L1, L2, L3, L4, L5, L6]; % Here good
            options = sdpsettings('solver',solverHead,'verbose',0);
            
            diagnostics = optimize(constraints,alph,options);

            % Solution
            P = value(X)^(-1);
            Pu = value(Z)^(-1);
            K = value(Y)*(value(X)^(-1));

            obj.K = K;
            obj.P = P; 
            obj.methodControl = 'optimal_robust';

        end
    end
end

