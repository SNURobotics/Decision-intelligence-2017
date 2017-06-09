clc;
clear;
close all;

%% read joint value and plot
% client = tcpip('localhost',9000,'InputBufferSize',10000,'OutputBufferSize',10000 );
% fopen(client);
% 
% % receive robot traj from server
% % joint
% while (1)
%     if client.BytesAvailable ~= 0
%         curr_data = fread(client, client.BytesAvailable);
%         char_data = char(curr_data);
%         display(char_data)
%         if (char_data(1) == 'J')
%             char_data = char_data(4:end);
%             ndiv = find(char_data == 'd');
%             n_data = round(ndiv/7);
%             qTrj = zeros(n_data, 6);
%             for i = 1:size(ndiv,1)
%                 n_data = fix((i-1) / 7) + 1;
%                 jointIdx = mod(i-1,7) + 1;
%                 if (jointIdx < 7)
%                     if i == 1
%                         qTrj(n_data,jointIdx) = str2num(char_data(1:ndiv(i)-1)');
%                     else
%                         qTrj(n_data,jointIdx) = str2num(char_data(ndiv(i-1)+1:ndiv(i)-1)');
%                     end
%                 end
%             end
%             figure()
%             plot(qTrj)
%         end
%     end
% end
%% read force value and plot
client = tcpip('localhost',9000,'InputBufferSize',10000,'OutputBufferSize',10000 );
fopen(client);

while (1)
    if client.BytesAvailable ~= 0
        curr_data = fread(client, client.BytesAvailable);
        char_data = char(curr_data);
        display(char_data)
        if (char_data(1) == 'M')
            char_data = char_data(4:end);
            ndiv = find(char_data == 'd');
            n_data = round(ndiv/6);
            qTrj = zeros(n_data, 6);
            for i = 1:size(ndiv,1)
                n_data = fix((i-1) / 6) + 1;
                jointIdx = mod(i-1,6) + 1;
                
                if i == 1
                    qTrj(n_data,jointIdx) = str2num(char_data(1:ndiv(i)-1)');
                else
                    qTrj(n_data,jointIdx) = str2num(char_data(ndiv(i-1)+1:ndiv(i)-1)');
                end
               
            end
            figure()
            plot(qTrj)
        end
    end
end
%% test
% joint
% char_data = [];
% char_data = [char_data; 'J'; '1'; 'd'];
% for i = 1:14
%     char_data = [char_data; num2str(0.1)'; 'd'];
% end
% if (char_data(1) == 'J')
%     char_data = char_data(4:end);
%     ndiv = find(char_data == 'd');
%     n_data = round(size(ndiv,1)/7);
%     qTrj = zeros(n_data, 6);
%     for i = 1:size(ndiv,1)
%         n_data = fix((i-1) / 7) + 1;
%         jointIdx = mod(i-1,7) + 1;
%         if (jointIdx < 7)
%             if i == 1
%                 qTrj(n_data,jointIdx) = str2num(char_data(1:ndiv(i)-1)');
%             else
%                 qTrj(n_data,jointIdx) = str2num(char_data(ndiv(i-1)+1:ndiv(i)-1)');
%             end
%         end
%     end
%     figure()
%     plot(qTrj)
% end

% force














