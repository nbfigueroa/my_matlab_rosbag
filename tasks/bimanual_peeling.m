%% Clear everything and Set Bag Directory

clear rosbag_wrapper;
clear ros.Bag;
clear all

bag_dir = '/home/nbfigueroa/Dropbox/Demonstrator_Year4/Peeling_demos_April29_bags/';
bags = dir(strcat(bag_dir,'*.bag'));

%% Set Topics of Interest

% Topic Names for Right Arm
right_pose_topic = '/KUKA_RightArm/Pose';
right_ft_topic   = '/hand/ft_sensor/netft_data';

% Topic Names for Left Arm
left_pose_topic  = '/KUKA_LeftArm/Pose';
left_ft_topic    = '/tool/ft_sensor/netft_data';

%% Read Topics from N demonstrations (bags)

N = length(bags);
data = {};
for ii=1:N    
    
    % Load one bag and visualize info
    bag = ros.Bag.load(strcat(bag_dir,bags(ii).name));
    bag.info()
    
    % Read topics from Right Arm
    clc;tic;
    fprintf('\n---> Reading Topics from Right Arm..');
    [r_pose, r_ft] = readPoseFtfromBag(bag, right_pose_topic, right_ft_topic);
    fprintf('..done\n');
        
    % Read topics from Left Arm
    fprintf('---> Reading Topics from Left Arm..');
    [l_pose, l_ft] = readPoseFtfromBag(bag, left_pose_topic, left_ft_topic);
    fprintf('..done\n');
    toc;
    
    % Make right/left data structs
    right.pose = r_pose;  right.ft   = r_ft;
    left.pose = l_pose;   left.ft    = l_ft;
    data{ii}.right = right; data{ii}.left = left;
    
    % Import TF Tree
    
    % Extract Important TFs
    
end
%% Data Pre-Processing

% At this point I could import the mat files -->

for jj=1:1
    % Pre-process data from right arm
    options.match_hz    = 1;
    options.smooth_fact = 0.005;
    options.do_plot     = 1;
    options.title_name  = ['Sequence: ', strrep(bags(jj).name,'.bag',''), ' ---- Arm: Right'];
    [X_r, t_r] = preProcessDataforLearning(data{jj}.right.pose, data{jj}.right.ft, ... 
        ros.Bag.load(strcat(bag_dir,bags(jj).name)), options);

    % Pre-process data from left arm
    options.smooth_fact = 0.001;
    options.title_name  = ['Sequence: ', strrep(bags(jj).name,'.bag',''), ' ---- Arm: Left'];
    [X_l, t_l] = preProcessDataforLearning(data{jj}.left.pose, data{jj}.left.ft,  ...
        ros.Bag.load(strcat(bag_dir,bags(jj).name)), options);
end