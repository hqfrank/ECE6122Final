function [data] = readBssFromFile(pathName, seedNum)
%READBSSFROMFILE Summary of this function goes here
%   Detailed explanation goes here
    numBssMax = 50;
    data = zeros(numBssMax, 3, seedNum * 6);
    fileList = dir(pathName);  % list all files in the folder
    files = fileList(3:end);   % the first two are '.' and '..' paths
    for i = 1:length(files)
        filename = files(i).name;
        [filepath,name,ext] = fileparts(filename);
        paras = strsplit(name,'_');
        index = str2num(paras{3}) - 499;
        numRelays = str2num(paras{4});
        density = idivide(numRelays, int32(1000));
        grid = idivide(str2num(paras{5}),int32(100)) - 1;
        if index > 100
            break;
        end 
        fileID = fopen(strcat(pathName , filename),'r');
        dataFile = fscanf(fileID,'%f,%f,%f');
        dataFile = reshape(dataFile, 3, []);
        data(1:length(dataFile(1,:)),:,(index-1)*6 + (density-1)*2 + grid) = dataFile';    
        fclose(fileID);
    end
end

