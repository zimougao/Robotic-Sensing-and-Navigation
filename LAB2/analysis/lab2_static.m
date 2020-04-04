clear all
clc
%% Load data
location = '/home/mou/eece5554_roboticssensing/LAB2/src/lab2_hw/rosbag/0209/stay_1.mat'; 
tag = 'robot';
imu = load_imudata_fil(location,tag);
