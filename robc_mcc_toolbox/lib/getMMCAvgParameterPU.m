function mmcStruct= getMMCAvgParameterPU(headString, Ts)

% Constant parametrs
sqrt23 = sqrt(2/3);
sqrt13 = sqrt(1/3);
muF = 1e-6;

switch headString
    case 'Tennet_2GW_Onshore'
        % Tennet 2GW Onshore MMC Parameters
        % Ts = 50e-6;                % [1/s]   Rated Sampling time
        fbase = 50;                 % [Hz]    Rated Nominal frequency
        wbase = 2*pi*fbase;         % [rad/s] Rated Angular frequency
        
        % Converter base values [MW, kV, kA, \Omega]
        Sbase = 1e3;                     % [MW] Rated Apparent power 
        V1ACbase = 400.0*sqrt23;         % [kV] Rated Primary AC Voltage
        V2ACbase = 275.0*sqrt23;         % [kV] Rated Secondary AC Voltage
        VDCbase = 525.0;                 % [kV] Rated DC Voltage
        
        [~,~,Z2ACbase,~,~] = getBaseValues(Sbase,...
                            V1ACbase, V2ACbase, VDCbase);
        
        % Per-unit values of equivalent arm resistance, inductance and 
        % capacitance 
        Rr = 0.15/Z2ACbase;            % [p.u]
        Lr = 0.05*wbase/(Z2ACbase);     % [p.u]
        
        % Per-unit values of equivalent transformer resistance and 
        % inductance 
        Rac = 0.006;            % [p.u]
        Lac = 0.18;             % [p.u]   

    case 'Tennet_2GW_Offshore'
        % Tennet 2GW Offshore MMC Parameters
        % Ts = 50e-6;                 % [1/s]   Rated Sampling time
        fbase = 50;                   % [Hz]    Rated Nominal frequency
        wbase = 2*pi*fbase;           % [rad/s] Rated Angular frequency
        
        % Converter base values [MW, kV, kA, \Omega]
        Sbase = 1e3;                      % [MW] Rated Apparent power 
        V1ACbase = 66.0*sqrt23;           % [kV] Rated Primary AC Voltage
        V2ACbase = 275.0*sqrt23;          % [kV] Rated Secondary AC Voltage
        VDCbase = 525.0;                  % [kV] Rated DC Voltage
        
        [~,~,Z2ACbase,~,~] = getBaseValues(Sbase,...
                            V1ACbase, V2ACbase, VDCbase);
        
        % Per-unit values of equivalent arm resistance, inductance and 
        % capacitance 
        Rr = 0.15/Z2ACbase;           % [p.u]
        Lr = 0.05*wbase/(Z2ACbase);     % [p.u]
        
        % Per-unit values of equivalent transformer resistance and 
        % inductance 
        Rac = 0.006;            % [p.u]
        Lac = 0.18;             % [p.u]   

    case 'CIGRE_B457_DCS1_Onshore'
        % CIGRE DCS1 2GW Onshore MMC Parameters
        % Ts = 50e-6;                % [1/s]   Rated Sampling time
        fbase = 50;                 % [Hz]    Rated Nominal frequency
        wbase = 2*pi*fbase;         % [rad/s] Rated Angular frequency
        
        % Converter base values [MW, kV, kA, \Omega]
        Sbase = 8e2;                     % [MW] Rated Apparent power 
        V1ACbase = 380.0*sqrt23;         % [kV] Rated Primary AC Voltage
        V2ACbase = 220.0*sqrt23;         % [kV] Rated Secondary AC Voltage
        VDCbase = 400.0;                 % [kV] Rated DC Voltage
        
        [~,~,Z2ACbase,~,~] = getBaseValues(Sbase,...
                            V1ACbase, V2ACbase, VDCbase);
        
        % Per-unit values of equivalent arm resistance, inductance and 
        % capacitance 
        Rr = 0.010/Z2ACbase;             % [p.u]
        Lr = 0.029*wbase/(Z2ACbase);     % [p.u]
        
        % Per-unit values of equivalent transformer resistance and 
        % inductance 
        Rac = 0.006;            % [p.u]
        Lac = 0.18;             % [p.u] 

        % Submodules Parameters
        N = 200;        % [] Number of submodule per arm
        CSM = 1e4*muF;  % [] Submodule capacitance
        
    case 'CIGRE_B457_DCS1_Offshore'
    otherwise
        warning('Model not founded')
end

mmcStruct = struct;

mmcStruct.Rr = Rr; 
mmcStruct.Lr = Lr; 
mmcStruct.Rac = Rac; 
mmcStruct.Lac = Lac; 
mmcStruct.w = 1; 

mmcStruct.wbase = wbase; 
mmcStruct.Sbase = Sbase; 
mmcStruct.fbase = fbase; 
mmcStruct.Z2ACbase = Z2ACbase; 

mmcStruct.Ts = Ts; 
mmcStruct.Ts_pu = Ts*wbase; 
end