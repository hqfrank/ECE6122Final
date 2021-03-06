function [] = distributedSchedulingSimulation(numSubframe,sdPairs,tDDown, tDUp, NR, IR, Alpha)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
% numSubframe: # of subframes the schedule runs
% sdPairs: [numPaths * 2], the source and destination bs Ids of each path
% tDDown: [numSubframe * numNodes], # of pkts per subframe downlink to sBSs
% tDUp: [numSubframe * numNodes], # of pkts per subframe uplink to mBS 
% NR: [numNodes * 1], # of radio chains on each base station
% IR: [numNodes * numNodes], interference relationship, IR(i,i) == 1
% Alpha: [numNodes * 1], scale parameter, Alpha(mBSId) == 0
%% System parameters
pktSize_byte = 1400;
slotsPerSubframe = 24;
nData = 22;
rPhyLink = 5;          % physical link rate 5 pkts per slot
numPaths = length(sdPairs(:,1));
sdPairs = sdPairs + 1; % increase the index of node by 1 to avoid node '0'
bsIds = unique(sdPairs); % unique base station Ids
numNodes = length(bsIds);
queueDown = zeros(numNodes, numNodes); % row is node, column is dst
% the value of queueDown(i,j), the queue size (in pkts) to j at node i
queueUp = zeros(numNodes, numNodes);   % row is node, column is src
% the value of queueUp(i,j), the queue size (in pkts) from j at i.
localTDDownPrev = zeros(1, numNodes);
localTDUpPrev   = zeros(1, numNodes);
localTDDownCur  = zeros(1, numNodes);
localTDUpCur    = zeros(1, numNodes);
% local schedule
localSchedule   = ones(1, numNodes).*nData;
%% Find the macro-cell BS
mBSId = setdiff(bsIds, sdPairs(:,2));  % mBS is never a destination in DL
assert(length(mBSId) == 1);            % only one macro-cell base station
tDDown(:,mBSId) = zeros(numSubframe,1);% mBS itself has no downlink traffic
tDUp(:,mBSId)   = zeros(numSubframe,1);% mBS itself has no uplink traffic
Alpha(mBSId,1)  = 0;                   % mBS has no input link

%% Update routing
[rTable, dirChild, height] = updateRouting(sdPairs, mBSId);  

%% Packet format
% (src, dst)

%% Network simulation
for i = 1:numSubframe % iterate each subframe
    fprintf('The %d-th subframe starts.\n', i);
    for j = 1:slotsPerSubframe % iterate each slot
        %% Control slot 1
        if j == 1
            %% generates new downlink packets at mBS
            queueDown(mBSId,:) = queueDown(mBSId,:) + tDDown(i,:);         % Downlink traffic always generates at mBS
            queueUp = queueUp + eye(numNodes) .* tDUp(i,:);                % Uplink traffic always generates at each small-cell BS
            %% send local traffic demand if it changes, update traffic demand on each node
            % the local traffic demand is its own demand + its children's
            for k = 1:numNodes
                kDirChild = dirChild(k,:) == 1;
                localTDDownCur(1,k) = tDDown(i,k) + sum(localTDDownPrev(1,kDirChild),2); % This is for its parent node to use
                localTDUpCur(1,k)   = tDUp(i,k)   + sum(localTDUpPrev(1,kDirChild),2);
            end
            localTDCur = (localTDDownCur + localTDUpCur);
            %% calculate local schedule based on updated traffic demand
            % Only non-leaf small-cell BS
            for k = 1:numNodes                
                if and(k ~= mBSId, sum(dirChild(k,:),2) > 0)
                    % parent link, child 1 link, child 2 link, ...
                    localNodesK = [k,find(dirChild(k,:))];
                    localTDInput = [(tDDown(i,k)+tDUp(i,k)),localTDCur(1,find(dirChild(k,:)))];
                    localIRInput = IR(localNodesK,localNodesK);
                    localAlphaInput = Alpha(localNodesK,1);
                    localResult = localSchedulerSmallCell(length(localNodesK), nData, NR(k), rPhyLink, localTDInput, localIRInput, localAlphaInput);
                    localTD  = [sum(localTDInput),localTDInput(2:end)];
                    numSlotsFloat = (localResult(end) .* localTD./rPhyLink).*localAlphaInput';
                    numSlotsInt   = ceil(numSlotsFloat);
                    recoverIndex  = find(numSlotsInt - numSlotsFloat > 0.95);
                    numSlotsInt(recoverIndex) = numSlotsInt(recoverIndex) - 1;
                    localSchedule(k) = numSlotsInt(1);
                end
            end
        %% Control slot 2    
        elseif j == 2
            %% Update the local traffic demand
            localTDDownPrev = localTDDownCur;
            localTDUpPrev   = localTDUpCur;
            %% determine the schedule
            for k = 1:numNodes
                %% macro-cell bs
                if k == mBSId
                    % get the direct child of mBS
                    localNodesM     = find(dirChild(mBSId,:));             % The indices of mBS's direct children
                    localQueueM     = zeros(length(localNodesM),1);        % The queue sizes (downlink + uplink) of mBS's direct children 
                    localQueueMDown = zeros(length(localNodesM),1);
                    localQueueMUp   = zeros(length(localNodesM),1);
                    % downlink queue: stores at mBS, queueDown(mBS,:)
                    for idx = 1:length(localNodesM)
                        curChild = localNodesM(idx);
                        localQueueMDown(idx) = sum(queueDown(mBSId, [curChild,find(rTable(mBSId,:)==curChild)]),2);
                    end
                    localQueueMUp   = sum(queueUp(localNodesM,:),2);
                    localQueueM     = localQueueMDown + localQueueMUp;
                    localScheduleM  = localSchedule(localNodesM);
                    localIRInput    = IR(localNodesM,localNodesM);
                    localAlphaInput = Alpha(localNodesM); 
                    localResultM = finalScheduleMacroCell(length(localNodesM), nData, NR(mBSId), rPhyLink, localQueueM, localIRInput, localAlphaInput, localScheduleM');
                    disp(localResultM);
                %% leaf small-cell bs    
                elseif sum(dirChild(k,:),2) > 0
                
                %% non-leaf small-cell bs    
                else
                end
            end
            
        else
            % data slot
            
        end
        
    end
    
end

end

