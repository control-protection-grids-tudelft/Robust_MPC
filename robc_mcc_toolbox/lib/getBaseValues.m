function [I1ACbase,I2ACbase,Z2ACbase,IDCbase,ZDCbase]=getBaseValues(Sbase, V1ACbase, V2ACbase, VDCbase)
% getBaseValues() computes the  base values knowing the base voltage and
% power.
% Author: Victor Daniel Reyes Dreke and Rahul Rane
% ------------------------------------------------------------------- %

% Constant parametrs
sqrt23 = sqrt(2/3);
% sqrt13 = sqrt(1/3);

% Computation
I1ACbase = sqrt23*(Sbase/V1ACbase);
I2ACbase = sqrt23^2*(Sbase/V2ACbase);
Z2ACbase = V2ACbase/I2ACbase;
IDCbase = sqrt23*(Sbase/VDCbase);
ZDCbase =  V2ACbase/IDCbase;


end