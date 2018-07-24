theta = 0:0.01:2*pi;
rho = zeros(size(theta));
for i = 1:length(theta)
    if (theta(i) < 7.5/180*pi || theta(i) > (2-7.5/180)*pi)
        rho(i) = 21.4;
    end
    if (theta(i) > 30/180*pi && theta(i) < 330/180*pi)
        rho(i) = -15;
    end
end
%Plot 2d graph
fmt = 'polar';
cutAngle = 0;
pattern(h, F, -180:180, cutAngle, 'PropagationSpeed', PS, 'Type', ...
    'directivity', 'CoordinateSystem', fmt );