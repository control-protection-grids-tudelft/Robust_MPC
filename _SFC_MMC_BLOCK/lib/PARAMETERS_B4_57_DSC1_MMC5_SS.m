% MMC1Data unit parameters
MMC1Data.VPri = 380;            % [kV] Primary transform side
MMC1Data.VdcBase = 400;         % [kV] DC Voltage

MMC1Data.SBase = 800;                     % [MVA]
MMC1Data.VPriBase = MMC1Data.VPri*r2o3;   % [kV] Primary transform side
MMC1Data.VSecBase = r2o3*220;                  % [kV] Secondary transform side


MMC1Data.IbaseP_pk =  r2o3*MMC1Data.SBase/MMC1Data.VPri;
MMC1Data.IbaseS_pk =  r2o3*MMC1Data.SBase/MMC1Data.VSecBase;
MMC1Data.Ibase = ((2/3)*MMC1Data.SBase)/(MMC1Data.VSecBase); % Peak-to-peak phase
MMC1Data.Zbase = (1*MMC1Data.VSecBase)/(MMC1Data.Ibase);
MMC1Data.Lbase = (MMC1Data.Zbase)/(2*pi*50);
MMC1Data.Vbase = (MMC1Data.VSecBase);
MMC1Data.Topology= "Half";      % []

MMC1Data.N= 200;                % [p.u.] Number of submodule per arm
MMC1Data.Lm= 0.029;             % [H]    Arm inductance
MMC1Data.Rm= 0.0;               % [Ohm]  Arm resistance
MMC1Data.w= 2*pi*50;            % [Ohm]  angular frequency

MMC1Data.CN= 1e4*muF;           % [F] Submodule capacitance
MMC1Data.DT= 4e3;               % [s]    Discharge Time Constant

MMC1Data.Lac= 0.18*MMC1Data.Lbase;  % [H]    AC-side inductance
MMC1Data.Rac= 0.0;                  % [Ohm]  AC-side resistance

MMC1Data.Ldc= 0.00;                 % [H]    DC-side inductance
MMC1Data.Rdc= 0.00;                 % [Ohm]  DC-side resistance

% Per unit
MMC1Data.VdcMaxPU = 1.05;
MMC1Data.VdcMinPU =  0.95;

MMC1Data.LmPU= MMC1Data.Lm/MMC1Data.Lbase;     % [p.u.]  Arm inductance
MMC1Data.RmPU= MMC1Data.Rm/MMC1Data.Zbase;     % [p.u.]  Arm resistance

MMC1Data.CNPU= [];                          % [p.u.] Submodule capacitance


MMC1Data.LacPU= 0.18;                       % [p.u.]  AC-side inductance
MMC1Data.RacPU= 0.001;                      % [p.u.]  AC-side resistance

MMC1Data.LdPUc= 0.00;                       % [p.u.]  DC-side inductance
MMC1Data.RdcPU= 0.00;                       % [p.u.]  DC-side resistance

MMC1Data.wPU= 1;                            % [p.u.]  Arm resistance