%% Clear everything and Set Bag Directory

clear rosbag_wrapper;
clear ros.Bag;
clear all

%---> MODIFY THESE DIRECTORIES
bag_dir = '/home/nbfigueroa/Desktop/kinesthetic_recordings/Yo/bags/pour_no_obst/';
data_dir = '/home/nbfigueroa/Desktop/kinesthetic_recordings/Yo/mat/pour_no_obst/';

bag_dir = '/home/nbfigueroa/Dropbox/gaga and borat/TRO-2017/Learning/bags/feetPlanar';
data_dir = '/home/nbfigueroa/Desktop/kinesthetic_recordings/Yo/mat/pour_no_obst/';

bags = dir(strcat(bag_dir,'*.bag'));

%% Set Topics of Interest
joint_state_topic = '/KUKA/joint_states';
ee_pose_topic = '/KUKA/Pose';

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
    [pose]         = readPosefromBag(bag, ee_pose_topic);    
    [joint_states] = readJointsfromBag(bag, joint_state_topic);    
    fprintf('..done\n');        
    toc;
    
    % Bag Times
    t_begin = bag.time_begin; t_end = bag.time_end;  
    
    % Make data structs
    kuka.pose           = pose;   
    kuka.joint_states   = joint_states;
    
    % Save as passive and active
    data{ii}.kuka = kuka;
    data{ii}.times = [t_begin t_end];
    data{ii}.name  = strrep(bags(ii).name,'.bag','');    
    
end

%% Save raw data to matfile
matfile = strcat(data_dir,'data.mat');
save(matfile,'data','bags','bag_dir')

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% START FROM HERE IF BAGS ALREADY Processed!!!
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Load Data from Mat File
matfile = strcat(data_dir,'data.mat');
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
    joint_data = joint_states(1:7,:);
    plot(joint_data');hold on;
    
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
    ee_traj = recording.kuka.pose(1:3,:);
    
    % Plot EE Trajectories
    plot3(ee_traj(1,:), ee_traj(2,:), ee_traj(3,:)); hold on;
end
xlabel('x');ylabel('y');zlabel('z');
grid on
title('Cartesian EE Trajectories')
