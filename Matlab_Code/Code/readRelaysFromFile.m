function [data] = readRelaysFromFile(pathName, dataType, seedNum, numRelays)
%READDATAFROMPATH Summary of this function goes here
%   Detailed explanation goes here
    numCandiRelaysMax = max(numRelays);
    data = zeros(numCandiRelaysMax, 3, seedNum * 3);
    fileList = dir(pathName);  % list all files in the folder
    files = fileList(3:end);   % the first two are '.' and '..' paths
    for i = 1:length(files)
        filename = files(i).name;
        [filepath,name,ext] = fileparts(filename);
        paras = strsplit(name,'_');
        index = str2num(paras{4}) - 499;
        density = str2num(paras{6});
        if index > 100
            break;
        end 
        fileID = fopen(strcat(pathName , filename),'r');
        dataFile = fscanf(fileID,'%f,%f,%f', [3, numRelays(density)]);
        data(1:numRelays(density),:,(index-1)*3 + density) = dataFile';    
        fclose(fileID);
    end
    

end

