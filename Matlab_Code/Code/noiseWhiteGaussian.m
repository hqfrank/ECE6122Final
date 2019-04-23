function noise_dBm = noiseWhiteGaussian(B_Hz)
%NOISEWHITEGAUSSIAN Summary of this function goes here
%   Detailed explanation goes here
k = 1.38 * 10 ^ (-23);
T = 290;
noise = k * T * B_Hz;

noise_dBm = 10 * log10(noise * 1000);

end

