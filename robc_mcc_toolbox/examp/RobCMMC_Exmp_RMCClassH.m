clear, clc,
% CT-SS Realization. Remark: Non-minimum phase system
Ac =  [0, 1; -6, -5];
% Bc =  [0, 0.1; 1, 0];
Bc =  [0.1;  1];
% Cc =  eye(2); 
Cc = [-1, 1];
Ec =  [0.1; 0.1];
% Ec =  eye(2); 

sys = ss(Ac,Bc,Cc,[]);

% DT-SS Realization
Ts =  0.01;                          % [s] Sampling time 
sysd = c2d(ss(Ac,Bc,Cc,[]),Ts);     % ZOH Discretization

% SS DT Matrices
A = sysd.A; B = sysd.B; C = sysd.C;

% Polytopic state and input constraints
Xbound = [-[1; 1], [1; 1]];                % [Xmax, Xmin] 
Ubound = [-[1; 1], [1; 1]]*10;             % [Umax, Umin] 
% Ubound = [-1, 1]*10;                     % [Umax, Umin] 

nx = size(Ac,1);
nu = size(Bc,2);
ny = size(Cc,1);
nw = size(Ec,2);


%% Filter / weighing functions design  MIMO case:

% Error 
ke = 1; we = 1;
Fe_ = tf(ke*we,[1 we]);
Eb = ones(ny,1);

Ebase_ = tf(Eb(1),1);

for i=1:ny-1
    Fe = tf(ke*we,[1 we]);
    Fe_ = append(Fe_,Fe);

    Ebase = tf(Eb(i+1),1);
    Ebase_ = append(Ebase_,Ebase);
end


% Reference
kr = 1; wr = 1;
Fr_ = tf(kr*wr,[1 wr]);
Rb = ones(ny,1);

Rbase_ = tf(Rb(1),1);

for i=1:ny-1
    Fr = tf(kr*wr,[1 wr]);
    Fr_ = append(Fr_,Fr);

    Rbase = tf(Rb(i+1),1);
    Rbase_ = append(Rbase_,Rbase);
end


% Input
ku = 0.1; wa = 1; wb = 1; 
Fu_ = tf(ku*[1 2*wa wa^2],[1 2*wb wb^2]);
Ub = ones(nu,1);

Ubase_ = tf(Rb(1),1);

for i=1:nu-1
    Fu = tf(ku*[1 2*wa wa^2],[1 2*wb wb^2]);
    Fu_ = append(Fu_,Fu);

    Ubase = tf(Ub(i+1),1);
    Ubase_ = append(Ubase_,Ubase);
end


% Disturbance
kd = 1; wd = 1;
Fd_ = tf(kd*wd,[1 wd]);

Db = ones(nw,1);
Dbase_ = tf(Db(1),1);

for i=1:nw-1
    Fd = tf(kd*wd,[1 wd]);
    Fd_ = append(Fd_,Fd);

    Dbase = tf(Db(i+1),1);
    Dbase_ = append(Dbase_,Dbase);
end



%%

rmc_c = RMCClass(A,B,C,[],[],Xbound,Ubound);

[K,gamma,~] = rmc_c.synthHInfty(Ac,Bc,Cc,[],Ec,...
                                            Fe_,Fd_,Fu_,Fr_,...
                                            Rbase_,Dbase_,Ubase_,Ebase_);


isstable(feedback(sys, K))

%%
Kd = c2d(K,Ts,'tustin');
isstable(feedback(sysd, Kd))
AKd = Kd.A;
Bkd = Kd.B;
Ckd = Kd.C;
Dkd = Kd.D;

%%
x0 = [0.5; -0.5];
xK0 = zeros(size(AKd,1),1);
Tsim = 1e4;
xvec = [x0, zeros(size(x0,1),Tsim)];
xKvec = [zeros(size(AKd,1),1), zeros(size(AKd,1),Tsim)] ;
yvec = Cc*xvec;
uvec = zeros(size(B,2),Tsim+1);

 for k=1:Tsim

     yvec(:,k) = Cc*xvec(:,k);
     uvec(:,k) = Ckd*xKvec(:,k)+Dkd* yvec(:,k);
     xvec(:,k+1)=(A)*xvec(:,k)+(B)*uvec(:,k); 
     xKvec(:,k+1)=(AKd)*xKvec(:,k)+(Bkd)*yvec(:,k);  
 end

