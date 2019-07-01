function [result] = localSchedulerSmallCell(numLinks, nd, NR, r, D, I, a)  %#codegen
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
% numLinks: the number of logical links attached to the current base
% station
% nd: number of data slots in a subframe;
% NR: number of radio chains;
% r: data rate of physical link in packets per slot
% D: traffic demand of each base station, [numLinks * 1], in packets per
% subframe
% I: the interference relationship between logical links [numLinks * numLinks]
% a: the scale parameter [numLinks * 1];
% if numLinks < 5 % the input has fewer effective paths
%     D = Di(1:numLinks,1);
%     I = Ii(1:numLinks,1:numLinks);
%     a = alphai(1:numLinks);
% else
%     D = Di;
%     I = Ii;
%     a = alphai;
% end

% optimization parameters
% In total there are numLinks + 1 variables
f = [zeros(1,numLinks),-1];  % minimize -S
intcon = [1:numLinks]; % the assigned number of slots is integer
% A1 * x <= B1 (capacity constraints)
A1 = [-r.* eye(numLinks),[sum(D);D(2:numLinks)]];
B1 = zeros(numLinks,1);
% A1 = [-r, 0, 0, 0, sum(D,2);
%      0, -r, 0, 0, D(2);
%      0, 0, -r, 0, D(3);
%      0, 0, 0, -r, D(4)];
% B1 = [0,0,0,0]';
% A2 * x <= B2 (interference constraints)
A2 = [I.*a', zeros(numLinks,1)];
B2 = ones(numLinks,1).*nd;
% A3 * x <= B3 (radio chain constraint)
A3 = [ones(1,numLinks),0];
B3 = nd * NR;
A = [A1;A2;A3];
B = [B1;B2;B3];
Aeq = [];
Beq = [];
lb = [ones(numLinks,1); 0];
ub = [nd./a; 1];
result = intlinprog(f,intcon,A,B,Aeq,Beq,lb,ub);
% 
% if numLinks < 5
%     resulto = [result;zeros(5-numLinks,1)];
% else
%     resulto = result;
% end
end

