function [capPath, demand, subtree] = getPathInfo(pathSeqPath,relayPath,bsPath, bsNeighborPath, pathPath, isSingleMBs)
%GETPATHINFO Summary of this function goes here
%   Detailed explanation goes here

% 3) Read path hops
fileID = fopen(pathSeqPath);
hops = fscanf(fileID,'%d\t%d',[2,Inf]);
pathSeq = hops(1,1:end-1);
hops = hops(2,1:end-1);
numNodeHop = hops + 2;
numPaths = size(numNodeHop,2);
fclose(fileID);

% 1) Read relays
fileID = fopen(relayPath);
relays = fscanf(fileID,'%f,%f,%f',[3,Inf]);
numRelays = size(relays,2);
relays = relays';
fclose(fileID);

% 2) Read BSs
fileID = fopen(bsPath);
BSs = fscanf(fileID,'%f,%f,%f',[3,Inf]);
numBSs = size(BSs,2);
BSs = BSs';
fclose(fileID);

% 7) Read BS neighbor info
fileID = fopen(bsNeighborPath);
bsNeighbor = ones(50,50).*-1;
% BsNeighbors = fscanf(fileID, '%d\t');
index = 1;
newline = sprintf('\r\n');
line = fgets(fileID);
while ischar(line)
    if strcmp(newline, line)
    else
        neighbors = strsplit(line,'\t');
        for i = 1:1:length(neighbors)-1
            bsNeighbor(index, i) = str2num(neighbors{i});
        end
    end
    line = fgets(fileID);
    index = index + 1;
end
fclose(fileID);

% 4) Read Paths
bsIndex = zeros(length(numPaths),2);
idxPath = 1;
isSrc = true;
fileID = fopen(pathPath);
paths = fscanf(fileID,'%d');
assert(length(paths) == sum(numNodeHop));
pNodes = zeros(length(paths),3);
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

fc_GHz = 60;
B_Hz = 2.16 * 10^9;
pt_dBm = 30;
gt_dBi = 21.4;
gr_dBi = 21.4;

noise_dBm = noiseWhiteGaussian(B_Hz);

% 6) Calculate path distance
idx = 1;
capPath = zeros(numPaths, 4);
for i = 1:numPaths
    n = numNodeHop(1, i);
    nodes = pNodes(idx:idx+n-1,:);
    % relaying path
    cp_length = zeros(max([1,n-2]),1);
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
    end
    cp_length_max = max(cp_length);
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

% 9) calculate the maximum D
% a. get the marco-cell bs
idxMBs = bsIndex(find(pathSeq == 0),1); % index is from 0
mBS = BSs(idxMBs + 1, :);               % the coordination of macro-cell BS
subtree = ones(numBSs, 1);
demand = zeros(numBSs, 2);
if isSingleMBs
    leaves = 1:numBSs;             % index + 1
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
    %
    demand
    subtree
else
end

% 5) Plot paths
idx = 1;
for i = 1:numPaths
    n = numNodeHop(1,i);
    nodes = pNodes(idx:idx+n-1,:);
    plot3(nodes(:,1),nodes(:,2),nodes(:,3),'LineWidth',1.5); hold on;
    idx = idx + n;
    if i == 1
        scatter3(nodes(1,1),nodes(1,2),nodes(1,3),40,'s','filled','MarkerFaceColor','r');
    else
        scatter3(nodes(end,1),nodes(end,2),nodes(end,3),30,'o','filled','MarkerFaceColor','b');
    end
    if n > 2
        scatter3(nodes(2:end-1,1),nodes(2:end-1,2),nodes(2:end-1,3),20,'^','filled','MarkerFaceColor','k');
    end
end
end

