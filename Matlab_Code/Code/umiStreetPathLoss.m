function pathLoss_dB = umiStreetPathLoss(d_3D_m, fc_GHz, los)
%3GPPUMISTREETPATHLOSS Summary of this function goes here
%   PL = 35.3log10(d3D)+21.3log10(fc)+22.4
h_BS_m = 10;
c_mps = 3 * 10^8;
fc_Hz = fc_GHz * 10 ^ 9;
d_BP_m = 4 * (h_BS_m - 1)^2 * fc_Hz / c_mps;

d_H_m = 0; % BS and UT are at the same height
h_UT_m = h_BS_m;
d_2D_m = d_3D_m;


% LOS
if d_2D_m < 10
    warning("The 2D distance between two nodes is shorter than 10 meters, and the 3GPP Umi Street Canyon path loss model is not accurate.");
elseif d_2D_m <= d_BP_m
    pathLoss_LOS_dB = 32.4 + 21 * log10(d_3D_m) + 20 * log10(fc_GHz);
elseif d_2D_m <= 5000
    pathLoss_LOS_dB = 32.4 + 40 * log10(d_3D_m) + 20 * log10(fc_GHz) - 9.5 * log10(d_BP_m^2 + d_H_m^2);
else
    warning("The 2D distance between two nodes is longer than 5000 meters, and the 3GPP Umi Street Canyon path loss model is not accurate.");
end
% sigma_SF = 4;

% NLOS
if d_2D_m < 10 || d_2D_m > 5000
    warning("The 2D distance between two nodes is shorter than 10 meters or longer than 5000m, and the 3GPP Umi Street Canyon path loss model is not accurate.");
else
    pathLoss_NLOS_dB = 35.3 * log10(d_3D_m) + 22.4 + 21.3 * log10(fc_GHz) - 0.3 * (h_UT_m - 1.5);       
end
% sigma_SF = 7.82;

if los
    pathLoss_dB = pathLoss_LOS_dB + 16 / 1000 * d_3D_m;
else
    pathLoss_dB = max(pathLoss_LOS_dB, pathLoss_NLOS_dB) + 16 / 1000 * d_3D_m;
end

end
