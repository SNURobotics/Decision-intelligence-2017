clc;
clear;
close all;

%% receive dataset from server
% client = tcpip('localhost',9000);
% fopen(client);
% 
% while (1)
%     fwrite(client,'G');
%     
%     if client.BytesAvailable ~= 0
%         curr_data = fread(client, client.BytesAvailable);
%         char_data = char(curr_data);
%         
%         ndiv = find(char_data == 'd');
%         
%         for i = 1:size(ndiv,1)
%             if i == 1
%                 tmp(i) = str2num(char_data(1:ndiv(i)-1)');
%             else
%                 tmp(i) = str2num(char_data(ndiv(i-1)+1:ndiv(i)-1)');
%             end
%         end
%     end 
% end



%% send desired robot dataset to server
robot_pos = [0.22 0.574 0.111];
char_robot_pos = [];
for i = 1:size(robot_pos,2)
    char_robot_pos = [char_robot_pos; num2str(robot_pos(i))'];
    char_robot_pos = [char_robot_pos; 'd']; 
end

robot_rot = [0.22 0.445 0.666; 0.582 1.235 2.1; 1.557 1.524 2.221];
[c1, c2] = size(robot_rot);
robot_rot = reshape(robot_rot, [1, c1*c2]);
char_robot_rot = [];
for i = 1:size(robot_rot,2)
    char_robot_rot = [char_robot_rot; num2str(robot_rot(i))'];
    char_robot_rot = [char_robot_rot; 'd']; 
end

robot_gripper = 1;
char_robot_gripper = [num2str(robot_gripper)];
char_robot_gripper = [char_robot_gripper; 'd']; 

% robot_ft = [0.2 0.33 0.54 1.22 30.22 20.5];
robot_ft = zeros(1,6);
char_robot_ft = [];
for i = 1:size(robot_ft,2)
    char_robot_ft = [char_robot_ft; num2str(robot_ft(i))'];
    char_robot_ft = [char_robot_ft; 'd'];
end

send_data = ['S'; num2str(2); char_robot_pos; char_robot_rot; char_robot_gripper; char_robot_ft; char_robot_pos; char_robot_rot; char_robot_gripper; char_robot_ft];

client = tcpip('localhost',9000);
fopen(client);

fwrite(client,send_data);

% pause;

% receive robot traj from server

while (1)
    
    if client.BytesAvailable ~= 0
        curr_data = fread(client, client.BytesAvailable);
        char_data = char(curr_data);
        char_data = char_data(2:end);
        ndiv = find(char_data == 'd');
        
        for i = 1:size(ndiv,1)
            if i == 1
                tmp(i) = str2num(char_data(1:ndiv(i)-1)');
            else
                tmp(i) = str2num(char_data(ndiv(i-1)+1:ndiv(i)-1)');
            end
        end
    end 
end

%% send vision dataset to server
% char_vision = [];
% for i = 1:3
%     char_vision = [char_vision; num2str(i)'; 'd'];
%     for j = 1:12
%         char_vision = [char_vision; num2str(i+0.01*j)'; 'd'];
%     end
% end
% char_vision = [char_vision; num2str(100)'; 'd'];
% for i = 1:3
%     for j = 1:6
%         char_vision = [char_vision; num2str(i+0.01*j)'; 'd'];
%     end
% end
% send_data = ['V'; char_vision]; 
% client = tcpip('localhost',9000);
% fopen(client);
% 
% fwrite(client,send_data);
% 
% pause;