clear variables;
close all;
clc;
datanum = 18;
load(['failure_data', num2str(datanum), '/sensorValTraj.txt'])
load(['failure_data', num2str(datanum), '/robotEndTraj_robotbase.txt'])
load(['failure_data', num2str(datanum), '/contactFValTraj.txt'])
colors = {'r', 'g', 'b'};
figure()
subplot(3,1,1)
plot(sensorValTraj(:,4:6))
subplot(3,1,2)
plot(contactFValTraj(:,4:6))
axis([0, size(contactFValTraj,1), -2,3])
subplot(3,1,3)
plot(robotEndTraj_robotbase(:,12))
datanumset = [18, 24];
for ii = 1:length(datanumset)
    datanum = datanumset(ii);
    load(['failure_data', num2str(datanum), '/sensorValTraj.txt'])
    load(['failure_data', num2str(datanum), '/robotEndTraj_robotbase.txt'])
    load(['failure_data', num2str(datanum), '/contactFValTraj.txt'])
    colors = {'r', 'g', 'b'};
    
    % figure()
    % plot(sensorValTraj(:,4:6))
    % figure()
    % plot(robotEndTraj_robotbase(:,12))
    minF = min(min(sensorValTraj(:,4:6)));
    maxF = max(max(sensorValTraj(:,4:6)));
    minP = min(min(robotEndTraj_robotbase(:,12)));
    maxP = max(max(robotEndTraj_robotbase(:,12)));
    H = figure();
    
    vidObj = VideoWriter(['contact_pos_',num2str(datanum),'.avi']);
    open(vidObj);
    h = [];
    h2 = [];
    for i = 1:size(sensorValTraj,1)
%         subplot(3,1,1)
%         for j = 1:3
%             plot(sensorValTraj(1:i,j+3)', colors{j})
%             hold on
%         end
%         if i == 1
%             for j = 1:3
%                 h(j) = plot([i, i],[sensorValTraj(i,j+3), sensorValTraj(i,j+3)], [colors{j}, 'o']);
%             end
%         else
%             for j = 1:3
%                 set(h(j), 'XData', [i, i], 'YData', [sensorValTraj(i,j+3), sensorValTraj(i,j+3)])
%             end
%         end
%         legend('f_x', 'f_y', 'f_z')
%         axis([0, size(sensorValTraj,1), minF*1.1 - maxF*0.1, maxF*1.1 - minF*0.1])
%         ylabel('F/T sensor (N)')
%         %     drawnow
         subplot(2,1,1)
        for j = 1:3
            plot(contactFValTraj(1:i,j+3)', colors{j})
            hold on
        end
        if i == 1
            for j = 1:3
                h2(j) = plot([i, i],[contactFValTraj(i,j+3), contactFValTraj(i,j+3)], [colors{j}, 'o']);
            end
        else
            for j = 1:3
                set(h2(j), 'XData', [i, i], 'YData', [contactFValTraj(i,j+3), contactFValTraj(i,j+3)])
            end
        end
%         legend('f_x', 'f_y', 'f_z')
        axis([0, size(contactFValTraj,1), -1.5, 3.5])
        ylabel('contact force (N)')
        subplot(2,1,2)
        plot(robotEndTraj_robotbase(1:i,12)','k')
        hold on
        if i == 1
            hpos = plot([i, i],[robotEndTraj_robotbase(i,12), robotEndTraj_robotbase(i,12)], 'ko');
        else
            set(hpos, 'XData', [i, i], 'YData', [robotEndTraj_robotbase(i,12), robotEndTraj_robotbase(i,12)])
        end
        ylabel('pos_z (m)')
        axis([0, size(sensorValTraj,1), minP*1.1 - maxP*0.1, maxP*1.1 - minP*0.1])
        % Write each frame to the file.
        currFrame = getframe(H);
        writeVideo(vidObj,currFrame);
    end
    % Close the file.
    close(vidObj);
end

close(vidObj);