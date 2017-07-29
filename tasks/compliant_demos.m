%% Clear everything and Set Bag Directory

clear rosbag_wrapper;
clear ros.Bag;
clear all

%---> MODIFY THESE DIRECTORIES
bag_dir = '../bags/feetPlanar/';
data_dir = '../mat/feetPlanar/';

bags = dir(strcat(bag_dir,'*.bag'));

%% Set Topics of Interest
joint_state_topic = '/joint_states';
ee_pose_topic     = '/KUKA/Pose';
ft_topic          = '/ft/ft_sensor/netft_data';

%% Read Topics from N demonstrations (bags)
N = length(bags);
data = {};
for ii=1:N    
    
    % Load one bag and visualize info
    bag = ros.Bag.load(strcat(bag_dir,bags(ii).name));
    bag.info()
    fprintf('Reading bag %s',bags(ii).name);
    
    % Read topics
    tic;
    fprintf('\n---> Reading Topics from Recording');   
    [pose, ft] = readPoseFtfromBag(bag, ee_pose_topic, ft_topic);
    [joint_states] = readJointsfromBag(bag, joint_state_topic);    
    fprintf('..done\n');        
    toc;
    
    % Bag Times
    t_begin = bag.time_begin; t_end = bag.time_end;  
    
    % Make data structs
    kuka.pose           = pose;   
    kuka.ft             = ft;
    kuka.joint_states   = joint_states;    
    
    % Save as passive and active
    data{ii}.kuka = kuka;
    data{ii}.times = [t_begin t_end];
    data{ii}.name  = strrep(bags(ii).name,'.bag','');    
    
end

%% Save raw data to matfile
matfile = strcat(data_dir,'raw_data.mat');
save(matfile,'data','bags','bag_dir')

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% START FROM HERE IF BAGS ALREADY Processed!!!
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Load Data from Mat File
matfile = strcat(data_dir,'raw_data.mat');
load(matfile);

%% Visualize Data
close all;

figure('Color',[1 1 1])
for i=1:length(data)
    
    % Extract desired trajectory
    recording = data{i};
    
    % Extract desired variables
    joint_states = recording.kuka.joint_states(1:7,:);

    % Plot Joint Trajectories
    plot(joint_states');hold on;
    
end
ylabel('rad')
xlabel('sample')
title('Joint Trajectories')
grid on;



figure('Color',[1 1 1])
for i=1:length(data)
    
    % Extract desired trajectory
    recording = data{i};
    
    % Extract desired variables
    ft_states = recording.kuka.ft(1:6,:);

    % Plot Joint Trajectories
    plot(ft_states');hold on;
    
end
ylabel('rad')
xlabel('sample')
title('FT Trajectories')
grid on;


figure('Color',[1 1 1])
for i=1:length(data)
    % Extract desired trajectory
    recording = data{i};
        
    % Extract desired variables
    ee_traj = recording.kuka.pose(1:3,:);
    
    % Plot EE Trajectories
    plot3(ee_traj(1,:), ee_traj(2,:), ee_traj(3,:)); hold on;
end
xlabel('x');ylabel('y');zlabel('z');
grid on
title('Cartesian EE Trajectories')
