function [C1] = collectCMatrixOnes(C)
%COLLECTCMATRIXONES collect the [i,j], such that C_{i,j}=1
%   C is the consecutive matrix.
length1 = size(C,1);
length2 = size(C,2);
C1 = zeros(length1*length2,2);
count = 1;
for i = 1:length1
    for j = 1:length2
        if C(i,j) == 1
            C1(count,:) = [i,j];
            count = count + 1;
        end
    end
end
if count > 1
    C1 = C1(1:count-1,:);
    filename = strcat('../Data/ConsecutiveLinkPairsOnly_', num2str(length1),'.mat');
    save(filename,'C1');
else
    C1 = 0;
end

