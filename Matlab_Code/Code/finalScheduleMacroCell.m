function [result] = finalScheduleMacroCell(numLinks, nd, NMR, r, Q, I, a, lS)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
% numLinks: the number of logical links attached to the current base
% station
% nd: number of data slots in a subframe;
% NMR: number of radio chains on the macro-cell BS;
% r: data rate of physical link in packets per slot
% Q: queue length of each base station, [numLinks * 1], in packets 
% I: the interference relationship between logical links [numLinks * numLinks]
% a: the scale parameter [numLinks * 1];
% lS: local schedule of small cell BSs attached to the macro-cell BS [numLinks * 1]

% optimization parameters
% In total there are numLinks + 1 variables
f = [zeros(1,numLinks),-1];  % minimize -S
intcon = [1:numLinks]; % the assigned number of slots is integer
% A1 * x <= B1 (capacity constraints)
A1 = [-r.* eye(numLinks),Q];
B1 = zeros(numLinks,1);
% A1 = [-r, 0, 0, 0, Q(1);
%       0, -r, 0, 0, Q(2);
%       0, 0, -r, 0, Q(3);
%       0, 0, 0, -r, Q(4)];
% B1 = [0,0,0,0]';
% A2 * x <= B2 (interference constraints)
A2 = [I.*a', zeros(numLinks,1)];
B2 = ones(numLinks,1).*nd;
% A3 * x <= B3 (radio chain constraint)
A3 = [ones(1,numLinks),0];
B3 = nd * NMR;
A = [A1;A2;A3];
B = [B1;B2;B3];
Aeq = [];
Beq = [];
lb = [ones(numLinks,1); 0];
ub = [lS./a; 1];
result = intlinprog(f,intcon,A,B,Aeq,Beq,lb,ub);

end

