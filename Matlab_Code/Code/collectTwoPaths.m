function [P2] = collectTwoPaths(numPaths)
%COLLECTTWOPATHS Collect the combination of two different paths
%   return a list of combinations
P2 = zeros(numPaths * numPaths - numPaths, 2);
count = 1;
for i = 1:numPaths
    for j = 1:numPaths
        if i ~= j
            P2(count,:) = [i,j];
            count = count + 1;
        end
    end
end

filename = strcat('../Data/PathCombinations_', num2str(numPaths),'.mat');
save(filename, 'P2');
end

