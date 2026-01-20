function [PAcVec,PBcVec,PCcVec,PAVec,PBVec,PCVec]=MMCValue2rho(MMC_ONSHORE,prv_max,method_d)
% MMCValue2rho()
% method_d = "euler";
[Ac_on, Bc_on, Cc_on] = getPlantModel(MMC_ONSHORE.Rr, MMC_ONSHORE.Rac, ...
                            MMC_ONSHORE.Lr, MMC_ONSHORE.Lac, ...
                            MMC_ONSHORE.w, 'OCC');


% SYS_On_DISC = c2d(ss(Ac_on, Bc_on, Cc_on,[]),MMC_ONSHORE.Ts_pu,method_d);
% DT SS Representation
if method_d =="euler"
    SYS_On_DISC.B = Bc_on*MMC_ONSHORE.Ts_pu;
    SYS_On_DISC.C = Cc_on*MMC_ONSHORE.Ts_pu;
    SYS_On_DISC.A = Ac_on*MMC_ONSHORE.Ts_pu+eye(size(Ac_on,1));
else
    SYS_On_DISC = c2d(ss(Ac_on, Bc_on, Cc_on,[]),MMC_ONSHORE.Ts_pu,method_d);
end
% Rearrangement
A_on = SYS_On_DISC.A; 
B_on = SYS_On_DISC.B;   
C_on = SYS_On_DISC.C;

[AcMax, BcMax, CcMax] = getPlantModel(MMC_ONSHORE.Rr+prv_max(1,1), ...
                                      MMC_ONSHORE.Rac+prv_max(1,2), ...
                                      MMC_ONSHORE.Lr+prv_max(1,3), ...
                                      MMC_ONSHORE.Lac+prv_max(1,4), ...
                                      MMC_ONSHORE.w+prv_max(1,5), 'OCC');



[AcMin, BcMin, CcMin] = getPlantModel(MMC_ONSHORE.Rr-prv_max(1,1), ...
                                      MMC_ONSHORE.Rac-prv_max(1,2), ...
                                      MMC_ONSHORE.Lr-prv_max(1,3), ...
                                      MMC_ONSHORE.Lac-prv_max(1,4), ...
                                      MMC_ONSHORE.w-prv_max(1,5), 'OCC');

if method_d =="euler"
    sysUMax.B = BcMax*MMC_ONSHORE.Ts_pu;
    sysUMin.B = BcMin*MMC_ONSHORE.Ts_pu;
    sysUMax.C = CcMax*MMC_ONSHORE.Ts_pu;
    sysUMin.C = CcMin*MMC_ONSHORE.Ts_pu;
    sysUMax.A = AcMax*MMC_ONSHORE.Ts_pu+eye(size(AcMax,1));
    sysUMin.A = AcMin*MMC_ONSHORE.Ts_pu+eye(size(AcMin,1));
else
    sysUMax = c2d(ss(AcMax, BcMax, CcMax,[]),MMC_ONSHORE.Ts_pu,method_d);
    sysUMin = c2d(ss(AcMin, BcMin, CcMin,[]),MMC_ONSHORE.Ts_pu,method_d);
end


PAcVec = max(abs(AcMax - Ac_on),abs(AcMin - Ac_on));
PBcVec = max(abs(BcMax - Bc_on),abs(BcMin - Bc_on));
PCcVec = max(abs(CcMax - Cc_on),abs(CcMin - Cc_on));

PAVec = max(abs(sysUMax.A - A_on),abs(sysUMin.A - A_on));
PBVec = max(abs(sysUMax.B - B_on),abs(sysUMin.B - B_on));
PCVec = max(abs(sysUMax.C - C_on),abs(sysUMin.C - C_on));


end