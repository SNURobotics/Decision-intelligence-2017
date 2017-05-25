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
% use test data
robot_pos1 = [-0.593602, -0.150047, 0.779937];
robot_rot1 =  [-0.064028 0.645112 -0.761401;
 -0.951107 -0.270455 -0.149167;
 -0.302153 0.714622 0.630887];
robot_rot1 = reshape(robot_rot1, 1,9);
robot_pos2 = [-0.461765, -0.004162, 0.788315];
robot_rot2 = [-0.124107 0.990070 -0.066026;
 -0.992242 -0.124315 0.000964;
 -0.007254 0.065634 0.997817];
robot_rot2 = reshape(robot_rot2, 1,9);
robot_pos3 = [-0.440500, -0.004500, 0.779000];
robot_rot3 = [0.000000 1.000000 0.000000;
 -1.000000 0.000000 -0.000000;
 -0.000000 -0.000000 1.000000];
robot_rot3 = reshape(robot_rot3, 1,9);

char_robot_pos1 = [];
for i = 1:size(robot_pos1,2)
    char_robot_pos1 = [char_robot_pos1; num2str(robot_pos1(i))'];
    char_robot_pos1 = [char_robot_pos1; 'd']; 
end
char_robot_pos2 = [];
for i = 1:size(robot_pos1,2)
    char_robot_pos2 = [char_robot_pos2; num2str(robot_pos2(i))'];
    char_robot_pos2 = [char_robot_pos2; 'd']; 
end
char_robot_pos3 = [];
for i = 1:size(robot_pos1,2)
    char_robot_pos3 = [char_robot_pos3; num2str(robot_pos3(i))'];
    char_robot_pos3 = [char_robot_pos3; 'd']; 
end
% robot_rot = [0.22 0.445 0.666; 0.582 1.235 2.1; 1.557 1.524 2.221];
% [c1, c2] = size(robot_rot);
% robot_rot = reshape(robot_rot, [1, c1*c2]);
char_robot_rot1 = [];
for i = 1:size(robot_rot1,2)
    char_robot_rot1 = [char_robot_rot1; num2str(robot_rot1(i))'];
    char_robot_rot1 = [char_robot_rot1; 'd']; 
end
char_robot_rot2 = [];
for i = 1:size(robot_rot1,2)
    char_robot_rot2 = [char_robot_rot2; num2str(robot_rot2(i))'];
    char_robot_rot2 = [char_robot_rot2; 'd']; 
end
char_robot_rot3 = [];
for i = 1:size(robot_rot1,2)
    char_robot_rot3 = [char_robot_rot3; num2str(robot_rot3(i))'];
    char_robot_rot3 = [char_robot_rot3; 'd']; 
end

robot_gripper = 0;
char_robot_gripper = [num2str(robot_gripper)];
char_robot_gripper = [char_robot_gripper; 'd']; 

% robot_ft = [0.2 0.33 0.54 1.22 30.22 20.5];
robot_ft = zeros(1,6);
char_robot_ft = [];
for i = 1:size(robot_ft,2)
    char_robot_ft = [char_robot_ft; num2str(robot_ft(i))'];
    char_robot_ft = [char_robot_ft; 'd'];
end




send_data = ['S'; num2str(3); char_robot_pos1; char_robot_rot1; char_robot_gripper; char_robot_ft; 
    char_robot_pos2; char_robot_rot2; char_robot_gripper; char_robot_ft;
    char_robot_pos3; char_robot_rot3; char_robot_gripper; char_robot_ft];

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