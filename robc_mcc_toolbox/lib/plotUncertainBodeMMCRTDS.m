function [Fig,MAG,PHASE,wout,idxPass] = plotUncertainBodeMMCRTDS(fileNames,plotFlag,rho)
% plotUncertainBodeMMC()
% 

%%
%  Reading table section
% Delimiter
% opts.Delimiter = ",";
% Specify file level properties
% opts.ExtraColumnsRule = "ignore";
% opts.EmptyLineRule = "read";
% Read table
% tempT = readtable(fileNames,opts);
tempT = readtable(fileNames);
tempT = table2array(tempT);
tempT = tempT(:,7:15);
% clear opts
%%

% Processing Frequency Data into Reponse Data [Magnitude, Phase] 
respT = zeros(size(tempT,1),4);

for  i=1:4
    respT(:,i) = 1./(tempT(:,2+2*(i-1))/1).*exp(-1j*tempT(:,3+2*(i-1))*pi/180);
end

% Processing Reponse Data [Magnitude, Phase] into Admittance Matrices
N = length(tempT);
Gjw =  zeros(2,2,N);
for i=1:N
    Gjw(:,:,i) = [respT(i,1), respT(i,2); respT(i,3), respT(i,4)];
end

% Frequency definition
% WLine = (tempT(:,1)*2*pi)/1; % respT(:,1)*2*pi is the frequency in rad/s.
WLine = (tempT(:,1))/1; % respT(:,1)*2*pi is the frequency in Hz.


% Frequency Response System Definition 
sysFrTf = idfrd(Gjw,WLine,0); 

% Input/Output Naming
sysFrTf.InputName = {'V^G_d','V^G_q'};
sysFrTf.OutputName = {'i^{\Delta}_d','i^{\Delta}_q'};


%%
% Generating plot
[mag,phase,wout] = bode(sysFrTf,WLine);
Nfreq = length(wout);

MAG = zeros(Nfreq,4); PHASE = zeros(Nfreq,4); 
idxPass = zeros(Nfreq,1);

for i=1:Nfreq
    % DD components
    MAG(i,1)= 20*log10(mag(1,1,i)); 
    % PHASE(i,1)= phase(1,1,i); % Original
    PHASE(i,1) = mod(phase(1,1,i) + 180, 360) - 180; % Wrap an angle to the range [-180°, 180°] 
    % DQ components
    MAG(i,2)= 20*log10(mag(1,2,i)); 
    % PHASE(i,2)= phase(1,2,i);
    PHASE(i,2) = mod(phase(1,2,i) + 180, 360) - 180; % Wrap an angle to the range [-180°, 180°] 
    % QD components
    MAG(i,3)= 20*log10(mag(2,1,i)); 
    % PHASE(i,3)= phase(2,1,i);
    PHASE(i,3) = mod(phase(2,1,i) + 180, 360) - 180; % Wrap an angle to the range [-180°, 180°]    
    % QQ components
    MAG(i,4)= 20*log10(mag(2,2,i)); 
    % PHASE(i,4)= phase(2,2,i);
    PHASE(i,4) = mod(phase(2,2,i) + 180, 360) - 180; % Wrap an angle to the range [-180°, 180°] 
    
    % Passivity Index
    % H_tf = (sys_tf + sys_tf');
    idxPass(i,1) = min(eig(Gjw(:,:,i)+Gjw(:,:,i)'));
end

if plotFlag==true
    Fig = figure;
    for i=1:4
        ax1 = subplot(2,4,i);
        semilogx(ax1,wout,MAG(:,i),'LineWidth',3.5);
        hold(ax1, "on")
        % xlabel(ax1,'Freq [Hz]')
        ylabel(ax1,'Mag [dB]')
        
        ax2 = subplot(2,4,4+i);
        semilogx(ax2,wout,PHASE(:,i),'LineWidth',3.5);
        hold(ax2, "on")
        xlabel(ax2,'Freq [Hz]')
        ylabel(ax2,'Phs [degree]')
    end
else
    Fig =[];
end

end