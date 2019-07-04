clc
clear all
close all

addpath('C:\Users\user\OneDrive\����\Decision-intelligence-2017\srLib2017\data\tbrrt_traj\');
%%
a = load('C:\Users\user\OneDrive\����\Decision-intelligence-2017\srLib2017\data\tbrrt_traj\tbrrt_traj_bfsmooth.txt');
figure()
plot(a)
grid on
%%
for i = 1:2:20
    str = strcat('C:\Users\user\OneDrive\����\Decision-intelligence-2017\srLib2017\data\tbrrt_traj\tbrrt_traj_iter', int2str(i));
    str = strcat(str, '.txt');
    a = load(str);
    figure()
    plot(a)
    grid on
end

%%
a = load('C:\Users\user\OneDrive\����\Decision-intelligence-2017\srLib2017\data\tbrrt_traj\tbrrt_traj_bfproj.txt');
figure()
plot(a)
grid on
%%
a = load('C:\Users\user\OneDrive\����\Decision-intelligence-2017\srLib2017\data\tbrrt_traj\tbrrt_traj_afproj.txt');
figure()
plot(a)
grid on