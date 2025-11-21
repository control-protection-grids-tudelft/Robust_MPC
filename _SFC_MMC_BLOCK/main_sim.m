% main_sim
% This script is to simulate the closed-loop response of the output current
% control of the MMC.
% 
% Dir: 
% lib\ containts all MATLAB functions used
% References:
% [1]
% --------------------------------------------------------------------- %
PARAMETERS_LOAD_SFC

prompt = "Select the type of simualtion MATLAB script [M] or"+...
    "Simulink [S]. The default value is [M].";
selText =  input(prompt,"s");

if isempty(selText)==true
    selText ="M";
end

% Uncertain ss matrices model as in () from [].
% A(p) = A0 + \Delta A(p) and  B(p) = B0 + \Delta B(p) 
Alpv =  @(p1,p2,p3,p4) A + [p1, p2; p3, p4]; 
Blpv =  @(p1,p2,p3,p4) B + [p1, p2; p3, p4]; 


if selText =="M"
% Closed-loop with MATLAB script
    Tsim = 1100;                    % [k] Amount of time steps 
    % References
    reference = [zeros(1,1e2), 0.5*ones(1,2e2), ...
                ones(1,1e2), -1*ones(1,4e2), 1*ones(1,3e2);
                zeros(1,2e2), 0.2*ones(1,1e2), ...
                ones(1,2e2), -0.5*ones(1,3e2), 0.5*ones(1,3e2)];

    % Initial conditions
    x_ss = reference; 
    u_ss = Kuss*reference; 
    x0 = x_ss(:,1)+0*[-0.2, 0.2]';
    
    % Number of uncertain models
    Nk = 5;    

    dp =  zeros(length(pvec), Nk-1);

    for i=1:length(pvec)
        if pvec(i) == 0
            dp(i,:) = zeros(1,Nk-1);
        else
            dp(i,:) = -pvec(i):(2.1*pvec(i)/(Nk-1)):pvec(i);% \delta \in ptemp
        end    
    end

    XUvec = cell(Nk,2);
    % ------------------------------------------------------------------ %
    % %
    for i=1:Nk
        % Simulation of the Nk possibles uncertain \Delta A
        if i==1 % Nominal dynamics
            [x,u]= simSystemSateFeedBack(A,B,K,x0,Tsim,x_ss,u_ss);
        else % Uncertain dynamics
            j = i-1;
            [x,u]= simSystemSateFeedBack(...
                Alpv(dp(1,j),dp(2,j),dp(3,j),dp(4,j)),...
                Blpv(dp(5,j),dp(6,j),dp(7,j),dp(8,j)),...
                K,x0,Tsim,x_ss,u_ss);
        end
        XUvec{i,1} = x;
        XUvec{i,2} = u;
    end    
elseif selText =="S"
% Closed-loop with Simulink model   
    Nk = 5;                 % [k] Number of uncertain models
    XUvec = cell(Nk,2);     % {Nk,2} Cell array 
    TSim = 10;              % [s] Simulation time
    Tsim = TSim/Ts;         % [k] Amount of time steps

    dp =  zeros(length(pvec), Nk-1);

    for i=1:length(pvec)
        if pvec(i) == 0
            dp(i,:) = zeros(1,Nk-1);
        else
            dp(i,:) = -pvec(i):(2.1*pvec(i)/(Nk-1)):pvec(i);% \delta \in ptemp
        end    
    end
    for i=1:Nk        
        if i==1
            A = Alpv(dp(1,i),dp(2,i),dp(3,i),dp(4,i));
            B = Blpv(dp(5,i),dp(6,i),dp(7,i),dp(8,i));
        else
            j = i-1;
            A = Alpv(dp(1,j),dp(2,j),dp(3,j),dp(4,j));
            B = Blpv(dp(5,j),dp(6,j),dp(7,j),dp(8,j));
        end

        out =  sim('SIM_SFC_MMC.slx');

        XUvec{i,1} = out.xkvec';
        XUvec{i,2} = out.ukvec';

        if i==Nk
            reference = out.rkvec';
        end
    end    
else
    disp('Other')
end

% --------------------------------------------------------------------- %
tShow = Tsim;
tIni = 0;

FigMainS=plotStateMPCv2(XUvec,Nk,Tsim,reference,tShow,tIni);
FigMainI=plotInputMPC(XUvec,Nk,Tsim,tShow,tIni);