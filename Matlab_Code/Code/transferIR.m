function [IRNode] = transferIR(IRLink,bsIndex)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
% bsIndex: src and dst ids of each path
% IRLink: interference relation between links
sdPairs = bsIndex + 1;
numNodes = size(sdPairs,1) + 1;
mBSId = setdiff(sdPairs(:,1), sdPairs(:,2));
IRNode = eye(numNodes);
IRNode(mBSId, mBSId) = 0;
for i = 1:numNodes-1
    for j = 1:numNodes-1
        if IRLink(i,j) == 1
            IRNode(sdPairs(i,2), sdPairs(j,2)) = 1;
        end
    end
end
end

