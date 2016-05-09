%% Clear everything and Set Bag Directory

clear rosbag_wrapper;
clear ros.Bag;
clear all

bag_dir = '/home/nadiafigueroa/Dropbox/Demonstrator_Year4/Peeling_demos_April29_bags/good/';
bags = dir(strcat(bag_dir,'*.bag'));

data_dir = '/home/nadiafigueroa/Dropbox/Demonstrator_Year4/Processed_data/April29/';

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
    fprintf('Reading bag %s',bags(ii).name);
    
    % Read topics from Right Arm (Passive)
    clc;tic;
    fprintf('\n---> Reading Topics from Right Arm..');
    [r_pose, r_ft] = readPoseFtfromBag(bag, right_pose_topic, right_ft_topic);    
    fprintf('..done\n');
        
    % Read topics from Left Arm (Active)
    fprintf('---> Reading Topics from Left Arm..');
    [l_pose, l_ft] = readPoseFtfromBag(bag, left_pose_topic, left_ft_topic);
    r_begin = bag.time_begin; r_end = bag.time_end;
    fprintf('..done\n');
    toc;
    
    % Bag Times
    t_begin = bag.time_begin; t_end = bag.time_end;  
    
    % Make right/left data structs
    right.pose = r_pose;  right.ft   = r_ft;
    left.pose = l_pose;   left.ft    = l_ft;
    
    % Save as passive and active
    data{ii}.passive = right; data{ii}.active = left;
    data{ii}.times = [t_begin t_end];
    data{ii}.name  = strrep(bags(ii).name,'.bag','');    
    
end

%% Save raw data to matfile
matfile = strcat(data_dir,'raw_data.mat');
save(matfile,'data','bags','bag_dir')

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% START FROM HERE IF BAGS ALREADY READ!!!
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Load Data from Mat File
matfile = strcat(data_dir,'raw_data.mat');
load(matfile);

%% Parse Relevant TF Information

bag = ros.Bag.load(strcat(bag_dir,bags(1).name));
% World Frame is the passive robot's base link
Passive_robot = eye(4);
tree = ros.TFTree(bag);
tree.allFrames()
world_frame  = 'calib_right_arm_base_link';

% Active Robot Base link
active_frame = 'calib_left_arm_base_link';    
tmp = tree.lookup(world_frame, active_frame, tree.time_begin + 1);
Active_robot = eye(4); Active_robot(1:3,end) = tmp.translation; 
Active_robot(1:3,1:3) = quaternion2matrix([tmp.rotation(4);tmp.rotation(1:3)]);

% Cutting Board Frames
cutting_frame_l = 'left_leg4_link';
tmp = tree.lookup(world_frame, cutting_frame_l, tree.time_begin + 1);
Cutting_board_l = eye(4); Cutting_board_l(1:3,end) = tmp.translation; 
Cutting_board_l(1:3,1:3) = quaternion2matrix([tmp.rotation(4);tmp.rotation(1:3)]);

cutting_frame_r = 'right_leg3_link';
tmp = tree.lookup(world_frame, cutting_frame_r, tree.time_begin + 1);
Cutting_board_r = eye(4); Cutting_board_r(1:3,end) = tmp.translation; 
Cutting_board_r(1:3,1:3) = quaternion2matrix([tmp.rotation(4);tmp.rotation(1:3)]);

% Hand FT (EE of Passive Arm)
Passive_flange = 'right_arm_flange_link';
hand_ft_frame  = 'Hand_ft';
tmp = tree.lookup(Passive_flange, hand_ft_frame, tree.time_begin + 1);
Hand_ft = eye(4); Hand_ft(1:3,end) = tmp.translation; 
Hand_ft(1:3,1:3) = quaternion2matrix([tmp.rotation(4);tmp.rotation(1:3)]);

% Tool FT (EE of Active Arm)
Active_flange  = 'left_arm_flange_link';
tool_ft_frame  = 'Tool_ft';
tmp = tree.lookup(Active_flange, tool_ft_frame, tree.time_begin + 1);
Tool_ft = eye(4); Tool_ft(1:3,end) = tmp.translation; 
Tool_ft(1:3,1:3) = quaternion2matrix([tmp.rotation(4);tmp.rotation(1:3)]);

%% Data Pre-Processing
close all
N = length(bags);
proc_data = {};

for jj=1:N
    
    % Pre-process data from passive arm
    options.match_hz    = 1;
    options.smooth_fact = 0.005;
    options.do_plot     = 1;
    options.tool_frame  = Hand_ft;
    options.base_frame  = [];
    options.title_name  = ['Sequence: ',data{jj}.name, ' ---- Arm: Passive'];
    [X_p, H_p, t_p] = preProcessEEData(data{jj}.passive.pose, data{jj}.passive.ft, ... 
        data{jj}.times, options);

    % Passive Arm Variables
    proc_data{jj}.passive.X = X_p;
    proc_data{jj}.passive.H = H_p;
    proc_data{jj}.passive.t = t_p;
    proc_data{jj}.passive.base_frame = eye(4);
    proc_data{jj}.passive.tool_frame = Hand_ft;
    
    % Pre-process data from active arm
    options.smooth_fact = 0.005;
    options.tool_frame  = Tool_ft;
    options.base_frame  = Active_robot;
    options.title_name  = ['Sequence: ',data{jj}.name, ' ---- Arm: Active'];
    [X_a, H_a, t_a] = preProcessEEData(data{jj}.active.pose, data{jj}.active.ft,  ...
        data{jj}.times, options);   
    
    % Active Arm Variables
    proc_data{jj}.active.X = X_a;
    proc_data{jj}.active.H = H_a;
    proc_data{jj}.active.t = t_a;    
    proc_data{jj}.active.base_frame = Active_robot;
    proc_data{jj}.active.tool_frame = Tool_ft;
    
    % Global Task Variables
    proc_data{jj}.name         = data{jj}.name;
    proc_data{jj}.board_lframe = Cutting_board_l;
    proc_data{jj}.board_rframe = Cutting_board_r;
    
    % Plot Trajectories with Reference Frames etc
    plotBimanualTrajectories(proc_data{jj})
    
end

% Save proc data to matfile
matfile = strcat(data_dir,'proc_data.mat');
save(matfile,'proc_data')



