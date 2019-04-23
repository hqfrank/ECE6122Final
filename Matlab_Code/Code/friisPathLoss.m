function [pathLoss_dB] = friisPathLoss(fc_Hz, d_3D_m)
%FRISSPATHLOSS Summary of this function goes here
%   Detailed explanation goes here
c_mps = 3 * 10^8;
lambda_m = c_mps/fc_Hz;
pathLoss_dB = 10 * log10(4 * pi * d_3D_m^2 / lambda_m^2);
end

