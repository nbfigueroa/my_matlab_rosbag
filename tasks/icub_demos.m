%% Clear everything and Set Bag Directory

clear rosbag_wrapper;
clear ros.Bag;
clear all; clc

%%---> MODIFY THESE DIRECTORIES
bag_dir = '../../lags-2018/bags/Test1_Oct24/';
data_dir = '../../lags-2018/mat/';
bags = dir(strcat(bag_dir,'*.bag'));
clear data

%% Set Topics of Interest
box_pose_topic       = '/Object/pose';
right_arm_ft_topic   = '/icub_right_arm_wrench';
left_arm_ft_topic    = '/icub_left_arm_wrench';

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
    ee_pose        = readPosefromBag(bag, box_pose_topic);
    fprintf('..done\n');        
    toc;
    
    % Bag Times
    t_begin = bag.time_begin; t_end = bag.time_end;  
    
    % Save as passive and active
    data{ii}.ee_pose       = ee_pose;
    data{ii}.dt            = ee_pose(8,end) - ee_pose(8,end-1);
    data{ii}.times         = [t_begin t_end];
    data{ii}.name          = strrep(bags(ii).name,'.bag','');    
    
end

%% Read TF frames from N demonstrations (bags)
tree = ros.TFTree(bag)
tree.allFrames()
world_frame  = 'world';

% Active Robot Base link
Box = '/Box';   
frames = [];
tmp = tree.lookup(world_frame, box_frame, tree.time_begin + 1);

%% Save raw data to matfile
matfile = strcat(data_dir,'demo1_iCub_raw_data.mat');
save(matfile,'data','bags','bag_dir')

%% Visualize EE position
figure('Color',[1 1 1])
for ii=4:N
chosen_demo  = ii; 
sample_size  = 3;

% Extract desired trajectory
recording = data{chosen_demo};

% Extract desired variables
ee_traj       = recording.ee_pose(1:3,1:sample_size:end);

% Plot EE Trajectories
color_traj = [rand rand rand]
scatter3(ee_traj(1,:), ee_traj(2,:), ee_traj(3,:), 7.5, 'MarkerEdgeColor','k','MarkerFaceColor',color_traj); hold on;
end
xlabel('$\xi_1$', 'Interpreter', 'LaTex', 'FontSize',15);
ylabel('$\xi_2$', 'Interpreter', 'LaTex','FontSize',15);
zlabel('$\xi_3$', 'Interpreter', 'LaTex','FontSize',15);
% legend({'Non-linear Task'},'Interpreter', 'LaTex','FontSize',15)
grid on
title('Cartesian EE Position Trajectories',  'Interpreter', 'LaTex','FontSize',15)
axis equal