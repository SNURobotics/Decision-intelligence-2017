clc; clear all; close all;

%%
datasize = 5;
data = cell(datasize,1);
%%
for iter = 1:5
    filename = strcat('ResultsChkeck00',num2str(iter-1),'.txt');
    fileID = fopen(filename,'r');
    formatSpec = '%s';
    A = fscanf(fileID,formatSpec);
    fclose(fileID);
    
    A = erase(A,'(ind)');
    A = split(A,'V');

    for i = 1:length(A)
        A{i} = split(A{i},'d');
    end

    A(1) = [];
    dummyIndex = [];
    for i = 1:length(A)
        if isnan(str2double(A{i}{1}(end-3:end))) || str2double(A{i}{1}) == 2
            dummyIndex(end+1) = i;
        end
    end
    for i = 1:size(dummyIndex)
        A(dummyIndex) = [];
    end
    
    data{iter} = cell(length(A),1);
    for i = 1:length(A)
        data{iter}{i} = cell(4,1);
        data{iter}{i}{1,1} = str2double(A{i}{1});
        tempR = zeros(1,9);
        for j = 2:10
            tempR(j-1) = str2double(A{i}{j});
        end
        data{iter}{i}{2,1} = reshape(tempR, 3, 3);
        tempX = zeros(1,3);
        for j = 11:13
            tempX(j-10) = str2double(A{i}{j});
        end
        data{iter}{i}{3,1} = tempX;
        tempF = [];
        for j = 14:18
            if ~isnan(str2double(A{i}{j}))
                tempF(1,end+1) = str2double(A{i}{j});
            end
        end
        data{iter}{i}{4,1} = tempF;
    end
end
%%
