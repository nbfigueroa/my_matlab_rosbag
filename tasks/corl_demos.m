%% Clear everything and Set Bag Directory

clear rosbag_wrapper;
clear ros.Bag;
clear all

%---> MODIFY THESE DIRECTORIES
bag_dir = '../../corl-2018/bags/';
data_dir = '../../corl-2018/mat/';

bags = dir(strcat(bag_dir,'*.bag'));

%% Set Topics of Interest
ee_pose_topic     = '/lwr/ee_pose';
gripper_topic     = '/SModelRobotOutput';

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
    ee_pose        = readPosefromBag(bag, ee_pose_topic, 1);
    gripper_state  = readGripperfromBag(bag, gripper_topic, ee_pose);
    fprintf('..done\n');        
    toc;
    
    % Bag Times
    t_begin = bag.time_begin; t_end = bag.time_end;  
    
    % Make data structs
    clear kuka
    kuka.ee_pose        = ee_pose;   
    kuka.gripper_state  = gripper_state;   
    kuka.joint_states   = joint_states;    
    
    % Save as passive and active
    data{ii}.kuka = kuka;
    data{ii}.times = [t_begin t_end];
    data{ii}.name  = strrep(bags(ii).name,'.bag','');    
    
end

%% Save raw data to matfile
matfile = strcat(data_dir,'demos_Sept9_2am_raw_data.mat');
save(matfile,'data','bags','bag_dir')

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% START FROM HERE IF BAGS ALREADY Processed!!!
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Load Data from Mat File
matfile = strcat(data_dir,'demos_Sept9_2am_raw_data.mat');
load(matfile);

%% Visualize EE position and velocity Data
figure('Color',[1 1 1])
chosen_demo = 3;

% Extract desired trajectory
recording = data{chosen_demo};

% Extract desired variables
ee_traj       = recording.kuka.ee_pose(1:3,:);
gripper_state = recording.kuka.gripper_state(1,:);

% Plot EE Trajectories
scatter3(ee_traj(1,gripper_state == 0), ee_traj(2,gripper_state == 0), ee_traj(3,gripper_state == 0), 7.5, 'MarkerEdgeColor','k','MarkerFaceColor',[0.5 .5 .5]); hold on;
scatter3(ee_traj(1,gripper_state == 255), ee_traj(2,gripper_state == 255), ee_traj(3,gripper_state == 255),  7.5, 'MarkerEdgeColor','k','MarkerFaceColor',[0 .95 .95]); hold on;

xlabel('$\xi_1$', 'Interpreter', 'LaTex', 'FontSize',15);
ylabel('$\xi_2$', 'Interpreter', 'LaTex','FontSize',15);
zlabel('$\xi_3$', 'Interpreter', 'LaTex','FontSize',15);
legend({'Return Primitive', 'Non-linear Primitive'},'Interpreter', 'LaTex','FontSize',15)
grid on
title('Cartesian EE Position Trajectories',  'Interpreter', 'LaTex','FontSize',15)
