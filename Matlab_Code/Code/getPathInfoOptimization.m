function [capPath, demand, totalDemand, subtree, Pf, Pl, sTreeSize, shareRadio, pathsAtM, IR, bsIndex] = getPathInfoOptimization(pathSeqPath,relayPath,bsPath, bsNeighborPath, pathPath, interfRelationPath, isSingleMBs, isInterfAllowed)
%GETPATHINFO Summary of this function goes here
%   Detailed explanation goes here

%% 1) Read number of relays, nodes in each path
fileID = fopen(pathSeqPath);
data = fscanf(fileID,'%d\t%d',[2,Inf]);
pathSeq = data(1,1:end-1);             % original sequence number of paths
numRelaysInPath = data(2,1:end-1);     % number of relays in each path
numNodesInPath = numRelaysInPath + 2;  % number of nodes in each path 
numPaths = size(numNodesInPath,2);     % number of paths
fclose(fileID);

%% 2) Read coordinations of relays
fileID = fopen(relayPath);
relays = fscanf(fileID,'%f,%f,%f',[3,Inf]);  % coords of candidate relays
numRelays = size(relays,2);                  % number of candidate relays
relays = relays';                            % each row a relay (x,y,z)
fclose(fileID);

%% 3) Read coordinations of BSs
fileID = fopen(bsPath);
BSs = fscanf(fileID,'%f,%f,%f',[3,Inf]);  % coords of base stations
numBSs = size(BSs,2);                     % number of base stations
BSs = BSs';                               % each row a BS (x,y,z)
fclose(fileID);

%% 4) Read BS neighbor info
fileID = fopen(bsNeighborPath);
bsNeighbor = ones(50,50).*-1;  % each element is the id of a neighbor BS
index = 1;
newline = sprintf('\r\n');
line = fgets(fileID);
while ischar(line)
    if strcmp(newline, line)
    else
        neighbors = strsplit(line,'\t');
        for i = 1:1:length(neighbors)-1  % the last cell is '\n'
            bsNeighbor(index, i) = str2num(neighbors{i});
        end
    end
    line = fgets(fileID);
    index = index + 1;
end
fclose(fileID);

%% 5) Read Paths
bsIndex = zeros(numPaths,2);
idxPath = 1;
isSrc = true;
fileID = fopen(pathPath);
paths = fscanf(fileID,'%d');      % each node's id in the path file
assert(length(paths) == sum(numNodesInPath));
pNodes = zeros(length(paths),3);  % coords of nodes in paths
for i = 1:length(paths)
    if (paths(i) >= numRelays)
        pNodes(i,:) = BSs(paths(i)-numRelays+1,:);
        if isSrc
            bsIndex(idxPath,1) = paths(i)-numRelays;
            isSrc = false;
        else
            bsIndex(idxPath,2) = paths(i)-numRelays;
            isSrc = true;
            idxPath = idxPath + 1;
        end
    else
        pNodes(i,:) = relays(paths(i)+1,:);
    end
end
fclose(fileID);

%% 6) Propagation parameters
fc_GHz = 60;
B_Hz = 2.16 * 10^9;
pt_dBm = 30;
gt_dBi = 26.4;
gr_dBi = 26.4;
noise_dBm = noiseWhiteGaussian(B_Hz);

%% 7) Calculate path distance
Pf = zeros(numPaths,1);  % the portion of first phy link in optimal schedule
Pl = zeros(numPaths,1);  % the portion of last phy link in optimal schedule
idx = 1;
capPath = zeros(numPaths, 4);
for i = 1:numPaths
    n = numNodesInPath(1, i);       % the number of nodes in the path
    nodes = pNodes(idx:idx+n-1,:);  % path nodes' id
    % relaying path
    cp_length = zeros(max([1,n-2]),1);  % consecutive link pair schedule length
    for j = 1:n-1
        src = nodes(j,:);
        dst = nodes(j+1, :);
        distance = sqrt(sum((dst - src).^2));
        pathLoss_LOS_dB = umiStreetPathLoss(distance, fc_GHz, true);
        cap_LOS_Gbps = shannonCapacity(pt_dBm, gt_dBi, gr_dBi, pathLoss_LOS_dB, noise_dBm, B_Hz * 10^-9);
        time_LOS_s = 1/cap_LOS_Gbps;
        if j <= max([1, n-2])
            cp_length(j) = cp_length(j) + time_LOS_s;
        end
        if j - 1 > 0
            cp_length(j-1) = cp_length(j-1) + time_LOS_s;
        end
        if j == 1
            Pf(i,1) = time_LOS_s;
        end
        if j == n-1
            Pl(i,1) = time_LOS_s;
        end
    end
    cp_length_max = max(cp_length);
    Pf(i,1) = Pf(i,1)/cp_length_max;  % calculate the portion of first phy link
    Pl(i,1) = Pl(i,1)/cp_length_max;  % calculate the portion of last phy link
    capPath(i, 1) = 1/cp_length_max;
    
    src = nodes(1,:);
    dst = nodes(end,:);
    distance = sqrt(sum((dst - src).^2));
    pathLoss_LOS_dB = umiStreetPathLoss(distance, fc_GHz, true);
    cap_LOS_Gbps = shannonCapacity(pt_dBm, gt_dBi, gr_dBi, pathLoss_LOS_dB, noise_dBm, B_Hz * 10^-9);
    capPath(i, 3) = cap_LOS_Gbps;
    pathLoss_NLOS_dB = umiStreetPathLoss(distance, fc_GHz, false);
    cap_NLOS_Gbps = shannonCapacity(pt_dBm, gt_dBi, gr_dBi, pathLoss_NLOS_dB, noise_dBm, B_Hz * 10^-9);
    capPath(i, 4) = cap_NLOS_Gbps;
    
    idxSrc = bsIndex(i,1);
    idxDst = bsIndex(i,2);
    cur_bsNeighbors = bsNeighbor(idxSrc+1,:);
    isLos = ~isempty(find(cur_bsNeighbors == idxDst));
    if isLos
        capPath(i,2) = capPath(i,3);
    else
        capPath(i,2) = capPath(i,4);
    end
    idx = idx + n;
end

%% 8) calculate the maximum D
% a. get the marco-cell bs
idxMBs = bsIndex(find(pathSeq == 0),1); % index is from 0
mBS = BSs(idxMBs + 1, :);               % the coordination of macro-cell BS
subtree = ones(numBSs, 1);
demand = zeros(numBSs, 2);
numBSsEffective = 0;
if isSingleMBs
    leaves = sort(unique(bsIndex)+1);             % index + 1
    numBSsEffective = length(leaves);
    for i = 1:length(bsIndex(:,1)) % iterate each logical link
        curBS = bsIndex(i,1) + 1;
        leaves = leaves(find(leaves~=curBS)); 
    end
    while length(leaves) > 0
        parent = [];
        for i = 1:length(leaves) % start from each leaf
            linkIdx = find(bsIndex(:,2) == (leaves(i)-1));
            temp_p = bsIndex(linkIdx,1);
            assert(length(temp_p) == 1, 'Node has more than 1 parents!');
            % update subtree and demand

            demand(leaves(i),:) = capPath(linkIdx,1:2)./subtree(leaves(i));
            subtree(temp_p + 1) = subtree(temp_p + 1) + subtree(leaves(i));
            % when parent node has only one child
            while (length(find(bsIndex(:,1) == temp_p)) == 1)
                linkIdx = find(bsIndex(:,2) == temp_p);
                next_p = bsIndex(linkIdx,1);
                assert(length(next_p) == 1, 'Node has more than 1 parents!');
                demand(temp_p+1,:) = capPath(linkIdx,1:2)./subtree(temp_p+1);
                subtree(next_p + 1) = subtree(next_p + 1) + subtree(temp_p+1);
                temp_p = next_p;
            end
            if temp_p ~= idxMBs 
                notLeaf = isempty(find(leaves==temp_p+1));
                if notLeaf
                    parent = [parent, temp_p + 1];
                end
            end
        end
        leaves = unique(parent);
    end
else
end

sTreeSize = zeros(numPaths,1);
shareRadio = zeros(numPaths,numPaths);
pathsAtM = zeros(numPaths,1);
for i = 1:numPaths
    dst = bsIndex(i,2);  % the index of the destination of path i
    sTreeSize = findSubtreeSize(dst, bsIndex, sTreeSize);
    shareRadio(i,find(bsIndex(:,1)==dst))=1;
    if bsIndex(i,1) == idxMBs
        pathsAtM(i) = 1;
    end
end

totalDemand = demand .* (numBSsEffective - 1);

%% 9) Read interference relationship
IR = eye(numPaths);
if isInterfAllowed
    fileID = fopen(interfRelationPath);
    data = fscanf(fileID, '%d\t%d\t%d\t%d',[4, Inf]);
    data = data';
    if ~isempty(data)
        numPairs = length(data(:,1));
        validIdx = ones(1,numPairs);
        interfPairs = [];
        for i = 1:numPairs
            if validIdx(i) > 0
                pathIndex1 = -1;
                pathIndex2 = -1;
                for j = 1:numPaths
                    if bsIndex(j,:) == data(i,1:2)
                        pathIndex1 = j;
                    end
                    if bsIndex(j,:) == data(i,3:4)
                        pathIndex2 = j;
                    end
                end
                interfPairs = [interfPairs;[pathIndex1, pathIndex2]];
                for j = i+1:1:numPairs
                    if data(j,1:2) == data(i,1:2)
                        if data(j,3) == data(i,3)
                            validIdx(j) = 0;
                        end
                    end
                    if data(j,1:2) == data(i,3:4)
                        if data(j,3) == data(i,1)
                            validIdx(j) = 0;
                        end
                    end
                    if data(j,3:4) == data(i,1:2)
                        if data(j,1) == data(i,3)
                            validIdx(j) = 0;
                        end
                    end
                    if data(j,3:4) == data(i,3:4)
                        if data(j,1) == data(i,1)
                            validIdx(j) = 0;
                        end
                    end
                end
            end
        end
        fclose(fileID);
        for i = 1:length(interfPairs(:,1))
            pathIdx1 = interfPairs(i,1);
            pathIdx2 = interfPairs(i,2);
            IR(pathIdx1, pathIdx2) = 1;
            IR(pathIdx2, pathIdx1) = 1;
        end
        for i = 1:length(IR(:,1))
            interfIdx = find(IR(i,:) == 1);
            assert(length(interfIdx) <= 3);
            if length(interfIdx) == 3
                tempIdx = interfIdx(interfIdx~=i);
                assert(length(tempIdx) == 2);
                assert(bsIndex(tempIdx(1),1)~=bsIndex(tempIdx(2),1));
            end
        end
    end
end
% 5) Plot paths
% idx = 1;
% for i = 1:numPaths
%     n = numNodeHop(1,i);
%     nodes = pNodes(idx:idx+n-1,:);
%     plot3(nodes(:,1),nodes(:,2),nodes(:,3),'LineWidth',1.5); hold on;
%     idx = idx + n;
%     if i == 1
%         scatter3(nodes(1,1),nodes(1,2),nodes(1,3),40,'s','filled','MarkerFaceColor','r');
%     else
%         scatter3(nodes(end,1),nodes(end,2),nodes(end,3),30,'o','filled','MarkerFaceColor','b');
%     end
%     if n > 2
%         scatter3(nodes(2:end-1,1),nodes(2:end-1,2),nodes(2:end-1,3),20,'^','filled','MarkerFaceColor','k');
%     end
% end
end

