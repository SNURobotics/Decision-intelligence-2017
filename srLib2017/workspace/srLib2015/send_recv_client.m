clc;
clear;
close all;

%% HYU waypoint 170814
% load('desired.mat');
load('desired_fix1.mat');
joint2 = zeros(6,1);
joint2(3) = -190/180*pi;
joint2(4) = pi/2;
joint2(5) = -100/180*pi;
I33 = reshape(eye(3),9,1);
p = zeros(3,1);
ft = zeros(6,1);

client = tcpip('localhost',9000,'InputBufferSize',10000,'OutputBufferSize',10000 );
fopen(client);

while(1)
    for idx = 1:length(save_snu)

        fwrite(client,'G');
        pause(0.001);
        send_data = ['R';'2';'d'];
        for i = 1:3
            send_data = [send_data; num2str(p(i))'; 'd'];
        end
        for i = 1:9
            send_data = [send_data; num2str(I33(i))'; 'd'];
        end
        send_data = [send_data; num2str(0)'; 'd'];
        for i = 1:6
            send_data = [send_data; num2str(ft(i))'; 'd'];
        end
        send_data = [send_data; num2str(0)'; 'd'];
        send_data = [send_data; num2str(0)'; 'd'];
        send_data = [send_data; num2str(0)'; 'd'];
        for i = 1:6
            send_data = [send_data; num2str(joint2(i))'; 'd'];
        end
        fwrite(client,send_data);
        pause(0.01);

        fwrite(client,save_snu{idx}.sendata);       % send S2d...
        pause(0.01);
       
        send_data2 = ['P';'2';'d'];
        for i = 1:3
            send_data2 = [send_data2; num2str(p(i))'; 'd'];
        end
        for i = 1:9
            send_data2 = [send_data2; num2str(I33(i))'; 'd'];
        end
        send_data2 = [send_data2; num2str(0)'; 'd'];
        for i = 1:6
            send_data2 = [send_data2; num2str(ft(i))'; 'd'];
        end
        send_data2 = [send_data2; num2str(0)'; 'd'];
        send_data2 = [send_data2; num2str(0)'; 'd'];
        send_data2 = [send_data2; num2str(0)'; 'd'];
        for i = 1:6
            send_data2 = [send_data2; num2str(joint2(i))'; 'd'];
        end
        fwrite(client,send_data2);
        pause(0.01);
    end
    send_data = ['A10']';
    fwrite(client,send_data);
    pause(0.01);
end



%% HYU waypoint
load('send_data.mat');
% 

I33 = reshape(eye(3),9,1);
p = zeros(3,1);
ft = zeros(6,1);
joint1 = zeros(6,1);
joint1(2) = pi/6;
joint1(3) = -220/180*pi;
joint1(4) = pi/2;
joint1(5) = -100/180*pi;
joint2 = zeros(6,1);
joint2(2) = -pi/2;
joint2(4) = pi/2;
joint2(5) = -pi/2;
% % % Trobot1torobot2 = ...
% % %     [-1 0 0 -1.44;
% % %     0 -1 0 0;
% % %     0 0 1 0;
% % %     0 0 0 1];
% % % tmp = [];
% % % newsenddata_set = {};
% % % for idx = 1:5
% % %     ndiv = find(senddata_set{idx} == 'd');
% % %     nway1 = str2num(senddata_set{idx}(ndiv(1)+1:ndiv(2)-1)');
% % %     nway2 = str2num(senddata_set{idx}(ndiv(2)+1:ndiv(3)-1)');
% % %     Tset1 = zeros(4,4,nway1);
% % %     Tset2 = zeros(4,4,nway2);
% % %     for i = 1:nway1
% % %         for j = 1:3
% % %             Tset1(j,4,i) = str2num(senddata_set{idx}(ndiv(3 + 19*(i-1) + j-1)+1:ndiv(3 + 19*(i-1) + j)-1));
% % %         end
% % %         for j = 1:9
% % %             c = fix((j-1) / 3) + 1;
% % %             r = mod(j-1, 3) + 1;
% % %             Tset1(r,c,i) = str2num(senddata_set{idx}(ndiv(3 + 19*(i-1) + 3 + j-1)+1:ndiv(3 + 19*(i-1) + 3 + j)-1));
% % %         end
% % %     end
% % %     for i = 1:nway2
% % %         Tset2(:,:,i) = 
% % %     end
% % %     newsenddata_set{idx} = [];
% % %     for i = 1:length(ndiv)
% % %         if i == 1
% % %             newsenddata_set{idx} = senddata_set{idx}(1:ndiv(1));
% % %         elseif and(i >= 3 + nway1 * 19 + 1, i < 3+nway1*19 + 1 + 12)
% % %         elseif and(i >= 3 + nway1 * 19 + 1 + 19, i < 3+nway1*19 + 1 + 19 + 12)
% % %         else
% % %             newsenddata_set{idx} = [newsenddata_set{idx}; senddata_set{idx}(ndiv(i)+1:ndiv(i+1))];
% % %         end
% % %     end
% % % end


client = tcpip('localhost',9000,'InputBufferSize',10000,'OutputBufferSize',10000 );
fopen(client);

while(1)
    for idx = 1:5
        for j = 1:2
            fwrite(client,'G');
            pause(0.001);
            send_data = ['R';'1';'d'];
            for i = 1:3
                send_data = [send_data; num2str(p(i))'; 'd'];
            end
            for i = 1:9
                send_data = [send_data; num2str(I33(i))'; 'd'];
            end
            send_data = [send_data; num2str(0)'; 'd'];
            for i = 1:6
                send_data = [send_data; num2str(ft(i))'; 'd'];
            end
            send_data = [send_data; num2str(0)'; 'd'];
            send_data = [send_data; num2str(0)'; 'd'];
            send_data = [send_data; num2str(0)'; 'd'];
            for i = 1:6
                send_data = [send_data; num2str(joint1(i))'; 'd'];
            end
            fwrite(client,send_data);
            pause(0.01);
        end
        fwrite(client,senddata_set{idx});
        pause(0.01);
       
        
        send_data = ['P';'1';'d'];
        for i = 1:3
            send_data = [send_data; num2str(p(i))'; 'd'];
        end
        for i = 1:9
            send_data = [send_data; num2str(I33(i))'; 'd'];
        end
        send_data = [send_data; num2str(0)'; 'd'];
        for i = 1:6
            send_data = [send_data; num2str(ft(i))'; 'd'];
        end
        send_data = [send_data; num2str(0)'; 'd'];
        send_data = [send_data; num2str(0)'; 'd'];
        send_data = [send_data; num2str(0)'; 'd'];
        for i = 1:6
            send_data = [send_data; num2str(joint1(i))'; 'd'];
        end
        fwrite(client,send_data);
        pause(0.01);
        send_data2 = ['P';'2';'d'];
        for i = 1:3
            send_data2 = [send_data2; num2str(p(i))'; 'd'];
        end
        for i = 1:9
            send_data2 = [send_data2; num2str(I33(i))'; 'd'];
        end
        send_data2 = [send_data2; num2str(0)'; 'd'];
        for i = 1:6
            send_data2 = [send_data2; num2str(ft(i))'; 'd'];
        end
        send_data2 = [send_data2; num2str(0)'; 'd'];
        send_data2 = [send_data2; num2str(0)'; 'd'];
        send_data2 = [send_data2; num2str(0)'; 'd'];
        for i = 1:6
            send_data2 = [send_data2; num2str(joint2(i))'; 'd'];
        end
        fwrite(client,send_data2);
        pause(0.01);
    end
    send_data = ['A10']';
    fwrite(client,send_data);
    pause(0.01);
end
% % fwrite(client,send_data2);
% % receive robot traj from server
% 
% % while (1)
% %     
% %     if client.BytesAvailable ~= 0
% %         curr_data = fread(client, client.BytesAvailable);
% %         char_data = char(curr_data);
% %         display(char_data)
% %         char_data = char_data(2:end);
% %         ndiv = find(char_data == 'd');
% %         
% %         for i = 1:size(ndiv,1)
% %             if i == 1
% %                 tmp(i) = str2num(char_data(1:ndiv(i)-1)');
% %             else
% %                 tmp(i) = str2num(char_data(ndiv(i-1)+1:ndiv(i)-1)');
% %             end
% %         end
% %     end 
% % end
% 
% %% test memory 
% send_data = ['R';'1';'d'];
% for i = 1:3
%     send_data = [send_data; num2str(p(i))'; 'd'];
% end
% for i = 1:9
%     send_data = [send_data; num2str(I33(i))'; 'd'];
% end
% send_data = [send_data; num2str(0)'; 'd'];
% for i = 1:6
%     send_data = [send_data; num2str(ft(i))'; 'd'];
% end
% send_data = [send_data; num2str(0)'; 'd'];
% send_data = [send_data; num2str(0)'; 'd'];
% for i = 1:6
%     send_data = [send_data; num2str(joint1(i))'; 'd'];
% end
% client = tcpip('localhost',9000,'InputBufferSize',10000,'OutputBufferSize',10000 );
% fopen(client);
% 
% 
% while(1)
%     fwrite(client,send_data);
% end
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
client = tcpip('localhost',9000,'InputBufferSize',10000,'OutputBufferSize',10000 );
fopen(client);

robot1_way1 = [2.33487e-16	1	6.12323e-17	-9.4698e-33	-6.12323e-17	1	1	-2.33487e-16	3.55679e-33	-0.2981	-0.1529	0.6789	];
robot1_way2 = [-1	4.22061e-09	-5.71878e-09	-4.22061e-09	-1	8.48822e-09	-5.71878e-09	8.48822e-09	1	-0.45378	-0.153346	0.8509	- 0.2];
robot1_way3 = [-1	3.68822e-09	-4.44613e-09	-3.68822e-09	-1	6.90343e-09	-4.44613e-09	6.90343e-09	1	-0.45378	0.2405	0.8509	- 0.2];


robot_pos1 =robot1_way1(10:12);
robot_rot1 = robot1_way1(1:9);
robot_pos2 =robot1_way2(10:12);
robot_rot2 = robot1_way2(1:9);
robot_pos3 =robot1_way3(10:12);
robot_rot3 = robot1_way3(1:9);


% robot2_way1 = [-0.210225	-0.952044	-0.2223	0.317276	-0.281512	0.905587	-0.924738	0.119847	0.361242	-1.06169	0.10515	0.756704	];
% robot2_way2 = [-1	-1.80851e-07	-2.87912e-13	1.80851e-07	-1	-1.83166e-13	-2.8796e-13	-1.83196e-13	1	-0.75378	-0.2395+0.1	0.8509-0.05		];
% robot2_way3 = [-1	-1.80851e-07	-2.99097e-13	1.80851e-07	-1	-1.84997e-13	-2.99102e-13	-1.84984e-13	1	-0.75378	0.2405-0.1	0.8509-0.05	];
% robot2_way4 = [-1	-1.80851e-07	-2.99097e-13	1.80851e-07	-1	-1.84997e-13	-2.99102e-13	-1.84984e-13	1	-0.75378	0.0	0.8509-0.05	];
% % 
% 
robot2_way1 = [-0.210225	-0.952044	-0.2223	0.317276	-0.281512	0.905587	-0.924738	0.119847	0.361242	-1.06169	0.10515	0.756704];
robot2_way2 = [-1	-1.80851e-07	-2.89722e-13	1.80851e-07	-1	-1.84016e-13	-2.89633e-13	-1.84024e-13	1	-0.75378	-0.128731	0.8509];
robot2_way3 = [-1	-1.80851e-07	-2.99097e-13	1.80851e-07	-1	-1.84997e-13	-2.99102e-13	-1.84984e-13	1	-0.75378	0.2405	0.8509		];


robot2_pos1 =robot2_way1(10:12);
robot2_rot1 = robot2_way1(1:9);
robot2_pos2 =robot2_way2(10:12);
robot2_rot2 = robot2_way2(1:9);
robot2_pos3 =robot2_way3(10:12);
robot2_rot3 = robot2_way3(1:9);
% robot2_pos4 =robot2_way4(10:12);
% robot2_rot4 = robot2_way4(1:9);

Tbusbarfinal = [reshape(robot2_rot3,3,3), robot2_pos3'];
Tbusbarfinal = [Tbusbarfinal; 0,0,0,1];

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

char_robot2_pos1 = [];
for i = 1:size(robot_pos1,2)
    char_robot2_pos1 = [char_robot2_pos1; num2str(robot2_pos1(i))'];
    char_robot2_pos1 = [char_robot2_pos1; 'd']; 
end
char_robot2_pos2 = [];
for i = 1:size(robot_pos1,2)
    char_robot2_pos2 = [char_robot2_pos2; num2str(robot2_pos2(i))'];
    char_robot2_pos2 = [char_robot2_pos2; 'd']; 
end
char_robot2_pos3 = [];
for i = 1:size(robot_pos1,2)
    char_robot2_pos3 = [char_robot2_pos3; num2str(robot2_pos3(i))'];
    char_robot2_pos3 = [char_robot2_pos3; 'd']; 
end
% char_robot2_pos4 = [];
% for i = 1:size(robot_pos1,2)
%     char_robot2_pos4 = [char_robot2_pos4; num2str(robot2_pos4(i))'];
%     char_robot2_pos4 = [char_robot2_pos4; 'd']; 
% end
char_robot2_rot1 = [];
for i = 1:size(robot_rot1,2)
    char_robot2_rot1 = [char_robot2_rot1; num2str(robot2_rot1(i))'];
    char_robot2_rot1 = [char_robot2_rot1; 'd']; 
end
char_robot2_rot2 = [];
for i = 1:size(robot_rot1,2)
    char_robot2_rot2 = [char_robot2_rot2; num2str(robot2_rot2(i))'];
    char_robot2_rot2 = [char_robot2_rot2; 'd']; 
end
char_robot2_rot3 = [];
for i = 1:size(robot_rot1,2)
    char_robot2_rot3 = [char_robot2_rot3; num2str(robot2_rot3(i))'];
    char_robot2_rot3 = [char_robot2_rot3; 'd']; 
end
% char_robot2_rot4 = [];
% for i = 1:size(robot_rot1,2)
%     char_robot2_rot4 = [char_robot2_rot4; num2str(robot2_rot4(i))'];
%     char_robot2_rot4 = [char_robot2_rot4; 'd']; 
% end
robot_gripper = 0;
char_robot_gripper = [num2str(robot_gripper)];
char_robot_gripper = [char_robot_gripper; 'd']; 
robot_gripper_on = 1;
char_robot_gripper_on = [num2str(robot_gripper_on)];
char_robot_gripper_on = [char_robot_gripper_on; 'd']; 
% robot_ft = [0.2 0.33 0.54 1.22 30.22 20.5];
robot_ft = zeros(1,6);
char_robot_ft = [];
for i = 1:size(robot_ft,2)
    char_robot_ft = [char_robot_ft; num2str(robot_ft(i))'];
    char_robot_ft = [char_robot_ft; 'd'];
end
% 
% client = tcpip('localhost',9000,'InputBufferSize',10000,'OutputBufferSize',10000 );
% 
% fopen(client);

send_data = ['S';'1'; 'd';num2str(3); 'd';char_robot_pos1; char_robot_rot1; char_robot_gripper; char_robot_ft; 
    char_robot_pos2; char_robot_rot2; char_robot_gripper; char_robot_ft;
    char_robot_pos3; char_robot_rot3; char_robot_gripper; char_robot_ft;'0';'d'];


send_data2 = ['S';'2'; 'd';num2str(3); 'd';char_robot2_pos1; char_robot2_rot1; char_robot_gripper; char_robot_ft; 
    char_robot2_pos2; char_robot2_rot2; char_robot_gripper; char_robot_ft;
    char_robot2_pos3; char_robot2_rot3; char_robot_gripper; char_robot_ft;'0';'d'];

send_data3 = ['S';'3'; 'd';num2str(3); 'd';num2str(3);'d';char_robot_pos1; char_robot_rot1; char_robot_gripper_on; char_robot_ft; 
    char_robot_pos2; char_robot_rot2; char_robot_gripper_on; char_robot_ft;
    char_robot_pos3; char_robot_rot3; char_robot_gripper_on; char_robot_ft;'0';'d';
    char_robot2_pos1; char_robot2_rot1; char_robot_gripper; char_robot_ft; 
    char_robot2_pos2; char_robot2_rot2; char_robot_gripper; char_robot_ft;
    char_robot2_pos3; char_robot2_rot3; char_robot_gripper_on; char_robot_ft;'0';'d'];

% 
fwrite(client,send_data3);
% while (1)
%     
%     if client.BytesAvailable ~= 0
%         curr_data = fread(client, client.BytesAvailable);
%         char_data = char(curr_data);
%         display(char_data)
%         char_data = char_data(2:end);
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
% % 
% % fwrite(client,send_data);

%% Send 'P'
I33 = reshape(eye(3),9,1);
p = zeros(3,1);
ft = zeros(6,1);
joint1 = zeros(6,1);
joint1(2) = pi/6;
joint1(3) = -220/180*pi;
joint1(4) = pi/2;
joint1(5) = -100/180*pi;
joint2 = zeros(6,1);
joint2(2) = -pi/2;
joint2(4) = pi/2;
joint2(5) = -pi/2;
send_data = ['P';'1';'d'];
for i = 1:3
    send_data = [send_data; num2str(p(i))'; 'd'];
end
for i = 1:9
    send_data = [send_data; num2str(I33(i))'; 'd'];
end
send_data = [send_data; num2str(0)'; 'd'];
for i = 1:6
    send_data = [send_data; num2str(ft(i))'; 'd'];
end
send_data = [send_data; num2str(0)'; 'd'];
send_data = [send_data; num2str(0)'; 'd'];
for i = 1:6
    send_data = [send_data; num2str(joint1(i))'; 'd'];
end

send_data2 = ['P';'2';'d'];
for i = 1:3
    send_data2 = [send_data2; num2str(p(i))'; 'd'];
end
for i = 1:9
    send_data2 = [send_data2; num2str(I33(i))'; 'd'];
end
send_data2 = [send_data2; num2str(0)'; 'd'];
for i = 1:6
    send_data2 = [send_data2; num2str(ft(i))'; 'd'];
end
send_data2 = [send_data2; num2str(0)'; 'd'];
send_data2 = [send_data2; num2str(0)'; 'd'];
for i = 1:6
    send_data2 = [send_data2; num2str(joint2(i))'; 'd'];
end

send_data3 = ['P';'3';'d'];
for i = 1:3
    send_data3 = [send_data3; num2str(p(i))'; 'd'];
end
for i = 1:9
    send_data3 = [send_data3; num2str(I33(i))'; 'd'];
end
send_data3 = [send_data3; num2str(0)'; 'd'];
for i = 1:6
    send_data3 = [send_data3; num2str(ft(i))'; 'd'];
end
send_data3 = [send_data3; num2str(0)'; 'd'];
send_data3 = [send_data3; num2str(0)'; 'd'];
for i = 1:6
    send_data3 = [send_data3; num2str(joint1(i))'; 'd'];
end

for i = 1:3
    send_data3 = [send_data3; num2str(p(i))'; 'd'];
end
for i = 1:9
    send_data3 = [send_data3; num2str(I33(i))'; 'd'];
end
send_data3 = [send_data3; num2str(0)'; 'd'];
for i = 1:6
    send_data3 = [send_data3; num2str(ft(i))'; 'd'];
end
send_data3 = [send_data3; num2str(0)'; 'd'];
send_data3 = [send_data3; num2str(0)'; 'd'];
for i = 1:6
    send_data3 = [send_data3; num2str(joint2(i))'; 'd'];
end

fwrite(client,send_data);
pause(0.01);

fwrite(client,send_data2);
pause(0.01);

%% Send S2 (second)
% client = tcpip('localhost',9000,'InputBufferSize',10000,'OutputBufferSize',10000 );
% fopen(client);
% send_data2 = ['S';'1'; 'd';num2str(1); 'd';char_robot2_pos4; char_robot2_rot4; char_robot_gripper; char_robot_ft; '0'];
% fwrite(client,send_data2);
% while (1)
%     
%     if client.BytesAvailable ~= 0
%         curr_data = fread(client, client.BytesAvailable);
%         char_data = char(curr_data);
%         display(char_data)
%         char_data = char_data(2:end);
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
%% receive robot traj from server
% 
% while (1)
%     if client.BytesAvailable ~= 0
%         curr_data = fread(client, client.BytesAvailable);
%         char_data = char(curr_data);
% %         display(char_data)
%         if (char_data(1) == 'M')
%            % display(char_data)
%             char_data = char_data(4:end);
%             ndiv = find(char_data == 'd');
%             n_data = round(size(ndiv,1)/6);
%             qTrj = zeros(n_data, 6);
%             for i = 1:size(ndiv,1)
%                 n_data = fix((i-1) / 6) + 1;
%                 jointIdx = mod(i-1,6) + 1;
%                 
%                 if i == 1
%                     qTrj(n_data,jointIdx) = str2num(char_data(1:ndiv(i)-1)');
%                 else
%                     qTrj(n_data,jointIdx) = str2num(char_data(ndiv(i-1)+1:ndiv(i)-1)');
%                 end
%                
%             end
%             figure()
%             plot(qTrj)
%         end
%     end
% end
% receive robot traj from server
% 
% while (1)
%     
%     if client.BytesAvailable ~= 0
%         curr_data = fread(client, client.BytesAvailable);
%         char_data = char(curr_data);
%         display(char_data)
%         char_data = char_data(2:end);
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

% send_data = [send_data; num2str(0)'; 'd'];
% for i = 1:6
%     send_data = [send_data; num2str(ft(i))'; 'd'];
% end
% for i = 1:6
%     send_data = [send_data; num2str(joint(i))'; 'd'];
% end
% 
% fwrite(client,send_data);
% 
% % receive robot traj from server
% 
% % while (1)
% %     
% %     if client.BytesAvailable ~= 0
% %         curr_data = fread(client, client.BytesAvailable);
% %         char_data = char(curr_data);
% %         display(char_data)
% %         char_data = char_data(2:end);
% %         ndiv = find(char_data == 'd');
% %         
% %         for i = 1:size(ndiv,1)
% %             if i == 1
% %                 tmp(i) = str2num(char_data(1:ndiv(i)-1)');
% %             else
% %                 tmp(i) = str2num(char_data(ndiv(i-1)+1:ndiv(i)-1)');
% %             end
% %         end
% %     end 
% % end

%% send vision dataset to server
% identity = eye(3);
% char_ori = reshape(identity, [1,9]);
% char_pos = [0, 0, -1;
%     0,0,-2];
Tx10 = eye(4);
Tx10(1,4) = 0.1;
% Tjig = [ 0.000  -1.000   0.000  -0.815  ;
% -0.540  -0.000  -0.841  -0.005  ;
%  0.841  -0.000  -0.540   0.869  ;
%  0.000   0.000   0.000   1.000  ;];
 Tjig = [0.000  -1.000   0.000  -0.815  ;
-1.000  -0.000   0.000  -0.032  ;
-0.000  -0.000  -1.000   0.869  ;
 0.000   0.000   0.000   1.000  ];
char_ori_jig = reshape(Tjig(1:3,1:3)',1,9);
char_pos_jig = Tjig(1:3,4);
Tbusbar1 = [ 0.000  -0.540   0.841   0.281  ;
-1.000   0.000   0.000   0.020  ;
-0.000  -0.841  -0.540   2.472  ;
 0.000   0.000   0.000   1.000  ;];
Tbusbar2 = Tx10 * Tbusbar1;
Tbusbar3 = Tx10 * Tbusbar2;

Trobot2_way2 = [-1	-1.80851e-07	-2.87912e-13	1.80851e-07	-1	-1.83166e-13	-2.8796e-13	-1.83196e-13	1	-0.75378	-0.2395+0.1	0.8509-0.05		];
Trobot2_way2 = reshape(Trobot2_way2, 3,4);
Trobot2_way2 = [Trobot2_way2; 0,0,0,1];
Tbusbar2gripper = [0.000   1.000   0.000   0.000  ;
 1.000  -0.000  -0.000   0.000  ;
 -0.000  0.000  -1.000   0.040  ;
 0.000   0.000   0.000   1.000  ];
TctCase2gripper = [ 1.000   0.000   0.000   0.006  ;
 0.000  -1.000  -0.000   0.032  ;
 -0.000  0.000  -1.000   0.010  ;
 0.000   0.000   0.000   1.000  ];
% Tbusbar4 = Trobot2_way2 * Tbusbar2gripper^-1;
Tbusbar4 = Trobot2_way2 * TctCase2gripper^-1;
Tbusbar4
char_ori_busbar1 = reshape(Tbusbar1(1:3,1:3)',1,9);
char_pos_busbar1 = Tbusbar1(1:3,4);
char_ori_busbar2 = reshape(Tbusbar2(1:3,1:3)',1,9);
char_pos_busbar2 = Tbusbar2(1:3,4);
char_ori_busbar3 = reshape(Tbusbar3(1:3,1:3)',1,9);
char_pos_busbar3 = Tbusbar3(1:3,4);
char_ori_busbar4 = reshape(Tbusbar4(1:3,1:3)',1,9);
char_pos_busbar4 = Tbusbar4(1:3,4);
char_vision = [];
char_ori = [char_ori_busbar1; char_ori_busbar2; char_ori_busbar3; char_ori_busbar4; char_ori_jig];
char_pos = [char_pos_busbar1'; char_pos_busbar2'; char_pos_busbar3'; char_pos_busbar4'; char_pos_jig'];
char_objID = [1,1,1,2,3];
for i = 1:5
    char_vision = [char_vision; num2str(char_objID(i))'; 'd'];
    for j = 1:12
        if j<=9
            char_vision = [char_vision;num2str(char_ori(i, j))';'d'];
        else
            char_vision = [char_vision;num2str(char_pos(i,j-9))';'d'];
        end
%         char_vision = [char_vision; num2str(i+0.01*j)'; 'd'];
    end
end
char_pos_obs = [0, 0, -3;
    0,0,-4];
char_size_obs = [0.5,0.5,0.5];

for i = 1:2
    char_vision = [char_vision; num2str(-1)'; 'd'];
    for j = 1:6
        if j<=3
            char_vision = [char_vision; num2str(char_pos_obs(i,j))'; 'd'];
        else
            char_vision = [char_vision; num2str(char_size_obs(j-3))'; 'd'];
        end
        
%         char_vision = [char_vision; num2str(i+0.01*j)'; 'd'];
    end
end
for i = 1:9
    char_vision = [char_vision; num2str(0.)'; 'd'];
end
send_data = ['V'; char_vision]; 
client = tcpip('localhost',9000,'InputBufferSize',10000,'OutputBufferSize',10000 );
fopen(client);

fwrite(client,send_data);

% pause;
% 
% char_pos(:,3) = char_pos(:,3) - 1;
% char_vision = [];
% for i = 1:5
%     char_vision = [char_vision; num2str(char_objID(i))'; 'd'];
%     for j = 1:12
%         if j<=9
%             char_vision = [char_vision;num2str(char_ori(i, j))';'d'];
%         else
%             char_vision = [char_vision;num2str(char_pos(i,j-9))';'d'];
%         end
% %         char_vision = [char_vision; num2str(i+0.01*j)'; 'd'];
%     end
% end
% char_pos_obs2 = [0, 1, -3;
%     1,0,-4];
% char_size_obs2 = [0.5,0.5,0.5];
% 
% for i = 1:2
%     char_vision = [char_vision; num2str(-1)'; 'd'];
%     for j = 1:6
%         if j<=3
%             char_vision = [char_vision; num2str(char_pos_obs2(i,j))'; 'd'];
%         else
%             char_vision = [char_vision; num2str(char_size_obs2(j-3))'; 'd'];
%         end
%         
% %         char_vision = [char_vision; num2str(i+0.01*j)'; 'd'];
%     end
% end
% for i = 1:9
%     char_vision = [char_vision; num2str(0.)'; 'd'];
% end;
% send_data2 = ['V'; char_vision]; 
% fwrite(client,send_data2);