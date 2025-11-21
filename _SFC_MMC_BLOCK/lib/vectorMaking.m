function vector_matrix = vectorMaking(A,xbounds)
% vectorMaking(A,xbounds) generates the set of vector that describe the
% convex hull that contains the system dynamics, see (22) in [1].
% -------------------------------------------------------------------- %
% Authors: (i) Victor Daniel Reyes Dreke           
% --------------------------------------------------------------------- %
% References:
% --------------------------------------------------------------------- %
% [1]   Dreke, Victor Daniel Reyes, and Mircea Lazar. "Practical deadbeat 
% mpc design via controller matching with applications in power 
% electronics." European Journal of Control 74 (2023): 100834.
% ===================================================================== %
n =  size(A,1);
binStr =  dec2bin([0:(2^n)-1]');
vector_matrix_index =  zeros(size(binStr,1),size(binStr,2)*2);

for i=1:size(binStr,1)
    for j=1:size(binStr,2)
        index =  [(j-1)*2+1:j*2];
        if binStr(i,j) == '1'
            vector_matrix_index (i,index) = [0 1];
        elseif binStr(i,j) == '0'
            vector_matrix_index (i,index) = [1  0];
        else
            disp('Somthing went wrong')
        end        
    end
end

vector_matrix_index  = boolean(vector_matrix_index);

vect_temp =  ones(2^n,1).*reshape(xbounds',[size(xbounds,1)*size(xbounds,2),1])';
vector_matrix = [];

for i=1:2^n
    vector_matrix =[vector_matrix;vect_temp(i,vector_matrix_index(i,:))];
end

end