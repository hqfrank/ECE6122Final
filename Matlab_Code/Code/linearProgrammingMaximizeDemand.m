function [x] = linearProgrammingMaximizeDemand(numPaths,C,Pf,Pl,B,SR,M,IR,isEnoughRadios,numRadiosSBS,RadiosM)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
%   C: numPath * 1
%   Pf = 0.5 .* ones(numPaths,1);
%   Pl = 0.5 .* ones(numPaths,1);
%   B = [3,2,1,3,2,1,3,2,1,3,2,1,3,2,1,3,2,1,1,1]'; % sTreeSize
%   SR: numPaths * numPaths  % shareRadio 
%   M: 1 * numPaths    % pathsAtM
if isEnoughRadios
    Radios = sum(SR,2)+1;
else
    Radios = numRadiosSBS .* ones(numPaths);
end

A1 = [-1.* eye(numPaths) .* (C./Pf), B];
b1 = zeros(numPaths,1);
I = eye(numPaths);
A2 = [IR .* (1./Pf)', zeros(numPaths,1)];
b2 = ones(numPaths,1);
A3 = [eye(numPaths) .* (Pl./Pf) + SR, zeros(numPaths,1)];
b3 = Radios;
A4 = [M, 0];
b4 = RadiosM;

f = [zeros(numPaths,1);-1];
A = [A1;A2;A3;A4];
b = [b1;b2;b3;b4];
lb = zeros(numPaths+1,1);
ub = [Pf;10];
Aeq = [];
beq = [];
x = linprog(f,A,b,Aeq,beq,lb,ub);
end

