function [Au]=uniqueColumns(A)
    % Initialize empty array to store unique matrices
    unique_mats = [];
    % Loop through each matrix in the list
    for i = 1:size(A,2)
        mat = A(:,i);
        
        % Check if the current matrix is unique
        is_unique = true;
        for j = 1:size(unique_mats,2)
            if isequal(mat, unique_mats(:,j))
                is_unique = false;
                break;
            end
        end
        
        % Add the current matrix to the unique_mats array if it's unique
        if is_unique
            unique_mats(:,end+1) = mat;
        end
    end
    % Display the unique matrices
    Au = unique_mats;
end