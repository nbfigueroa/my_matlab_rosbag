%% Clear everything and Set Bag Directory

clear rosbag_wrapper;
clear ros.Bag;
clear all

bag_dir = '/home/nbfigueroa/Desktop/kinesthetic_recordings/Yo/bags/';
data_dir = '/home/nbfigueroa/Desktop/kinesthetic_recordings/Yo/mat/';

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

% Select Bag
recording = data{1};

% Extract desired variables
joint_data = recording.kuka.joint_states(1:7,:);
ee_traj    = recording.kuka.pose(1:3,:);

% Plot Joint Trajectories
joint_data = joint_states(1:7,:);
figure('Color',[1 1 1])
plot(joint_data')
ylabel('rad')
xlabel('sample')
title('Joint Trajectories')
grid on;

% Plot EE Trajectories
ee_traj = pose(1:3,:);
figure('Color',[1 1 1])
plot3(ee_traj(1,:), ee_traj(2,:), ee_traj(3,:))
xlabel('x');ylabel('y');zlabel('z');
grid on
title('Cartesian EE Trajectories')
