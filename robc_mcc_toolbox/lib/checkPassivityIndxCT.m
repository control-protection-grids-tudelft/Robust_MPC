function [H_tf,idxPass,flagPass,win] = checkPassivityIndxCT(A,B,C,D,win)
% [H_tf,idxPass,flagPass,win] = checkPassivityIndxCT(A,B,C,D,win) function
% to examine passivity of LTI systems based on the real positive lemma [1].
% -----------------------------------
% Inputs:   - (A,B,C,D) are the state-space matrices (standard notation)
%           - win maximum frequency to scan.
% Outputs:  - H_tf is complementary transfer function. 
%           - idxPass is the vector of insertion indeces for all frequencies.
%           - flagPass (true) if all insertion indeces are positive. 
% ------------------------------------
% Author: Victor Daniel Reyes Dreke
% ------------------------------------
% References: 
% [1] Bao, J., & Lee, P. L. (2007). Process control: the passive 
% systems approach. London: Springer London.
% ------------------------------------

nx = size(B,1); 
nu = size(B,2); 
ny = size(C,1);

sys_ss = ss(A,B,C,D);

sys_tf = tf(sys_ss);

H_tf = (sys_tf + sys_tf');

if nargin>=5 

    if isempty(win)==true
        win = [0, logspace(-2,5,1000)]';
    end

    idxPass=zeros(length(win),1);
    % idxPass2=zeros(length(win),1);
    flagPass = true;

    for i=1:length(win)
        idxPass(i,1) = min(eig(freqresp(H_tf,win(i))));
        % Both are correct. 
        % idxPass2(i,1) = min(eig(freqresp(sys_tf,win(i))+ freqresp(sys_tf,win(i))'));
        if idxPass<0
            flagPass = false;
        end
    end
else

idxPass =[]; 

flagPass =[];

end


end