function [MMC1Data] = parametersLibary(NameExample)
% parametersLibary(NameExample) loading parameters RSCAD model.
%
% 

% Intialization
r2o3 = sqrt(2/3);
r1o3 = sqrt(1/3);
muF = 1e-6;
%
ListModels = {'B4_57_DSC1_MMC5_SS';'B4_57_DSC1_MMC5_SS'}; 
%
NameExample = string(NameExample);
%
switch NameExample
    case "B4_57_DSC1_MMC5_SS"
        % disp('Nica')  
        PARAMETERS_B4_57_DSC1_MMC5_SS
    otherwise
        warning("Model not found.")
        fprintf('Select on of the model from this list:\n');
        for i=1:length(ListModels) 
            fprintf('%s \n',ListModels{i});
        end
end
end