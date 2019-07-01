function [sTreeSize] = findSubtreeSize(root,bsIndex,sTreeSize)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
numNodes = 1;
pathIdx = find(bsIndex(:,2) == root);
assert(length(pathIdx) == 1);
childrenIdx = find(bsIndex(:,1) == root);  % find links starting at root
if ~isempty(childrenIdx)
    for i = 1:length(childrenIdx)
        dst = bsIndex(childrenIdx(i),2);
        if sTreeSize(childrenIdx(i)) > 0
            numNodes = numNodes + sTreeSize(childrenIdx(i));
        else
            sTreeSize = findSubtreeSize(dst, bsIndex, sTreeSize);
            numNodes = numNodes + sTreeSize(childrenIdx(i));
        end
    end
end
sTreeSize(pathIdx) = numNodes;
end

