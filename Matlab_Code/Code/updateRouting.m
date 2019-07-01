function [rTable, dirChild, height] = updateRouting(bsPairs,mBSId)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
% rTable: 
%   [numNodes * numNodes], rTable(i,j) give the node id of the next hop 
%   from node i to node j. rTable(i,i) == 0;
% bsPairs:
%   [numLinks * 2], each row is the src and dst ids of a path; note that
%   ids have been added 1 already.

numPaths = size(bsPairs,1);     % total # of paths (logical links)
numNodes = numPaths + 1;        % total # of base stations, tree topology
rTable   = zeros(numNodes,numNodes);  % routing table.
subTree  = eye(numNodes, numNodes);   % Indexing array for subtree
% if subTree(i,j) == 1, node j is in the subtree rooted at node i
% subTree(i,i) == 1, node i is in its subtree
dirChild = zeros(numNodes, numNodes); % Indexing array for direct child
% if dirChild(i,j) == 1, node j is the direct child node of node i
height   = zeros(numNodes,1);

%% Update the subTree and dirChild info, and update the uplink routing
leaves = setdiff(bsPairs(:,2), bsPairs(:,1)); % leaf nodes, only as dst
while ~isempty(leaves)       % leave is not empty
    parents = [];            % the parent nodes
    for i = 1:length(leaves) % iterates each leaf node
        leaf = leaves(i);
        tempIdx = find(bsPairs(:,2)==leaf); % each node is dst once
        if isempty(tempIdx)  % current leaf is mBS 
            continue;
        elseif length(tempIdx) > 1
            disp(leaf)
            disp(bsPairs)
            disp(tempIdx)
        end
        parent = bsPairs(tempIdx, 1);
        rTable(leaf, mBSId) = parent; % uplink routing from leaf to mBS
        dirChild(parent, leaf) = 1;   % leaf is a direct child of parent
        % all the node in subTree(leaf,:) are also in subTree(parent,:)
        subTree(parent,subTree(leaf,:)==1) = 1;
        parents = [parents, parent];
        parents = unique(parents);
    end
    leaves = parents;
end
%% update the downlink routing
for i = 1:numNodes
    % for each direct child of i, all the nodes in its subtree has a
    % route with the next hop as that direct child
    for j = 1:numNodes
        if dirChild(i,j) == 1 % j is a direct child of i
            % all nodes in the subTree of j
            rTable(i, subTree(j,:) == 1) = j;
        end
    end
end
%% Update height info
visited = [];
nextLevel = mBSId;
while ~isempty(nextLevel)
    if ~isempty(visited)
        height(visited) = height(visited) + 1;
    end
    newLevel = [];
    height(nextLevel) = 1;
    for i = 1:length(nextLevel)
        cur = nextLevel(i);
        visited = [visited, cur];
        newLevel = [newLevel, find(dirChild(cur,:))];
    end
    nextLevel = newLevel;
end
%% Print out
fprintf('The routing table is:\n');
disp(rTable);


end

