function cap_Gbps = shannonCapacity(pt_dBm, gt_dBi, gr_dBi, pL_dB, n_dBm, B_GHz)
%SHANNONCAPACITY Summary of this function goes here
%   snr = pr - noise
%       = pt + Gt + Gr - PL - noise
snr_dB = pt_dBm + gt_dBi + gr_dBi - pL_dB - n_dBm - 10;

cap_Gbps = B_GHz * log2(1 + 10^(snr_dB/10));


end

