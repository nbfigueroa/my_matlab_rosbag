%% Clear everything and Set Bag Directory

clear rosbag_wrapper;
clear ros.Bag;
clear all; clc

%%---> MODIFY THESE DIRECTORIES
bag_dir = '../../jmlr-2019/bags/combined/';
data_dir = '../../jmlr-2019/mat/';

bags = dir(strcat(bag_dir,'*.bag'));

%% Set Topics of Interest
ee_pose_topic       = '/lwr/ee_pose';
ee_ft_topic         = '/ft_sensor/netft_data';
joint_states_topic  = '/lwr/joint_states';

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
    [pose, ft]              = readPoseFtfromBag(bag, ee_pose_topic, ee_ft_topic, 1);
    [joint_states, joint_t] = readJointsfromBag(bag, joint_states_topic);
    
    fprintf('..done\n');        
    toc;
    
    final_time = round(52*100);
    
    
    % Make data structs
    if ii == 2
        raw_data{ii}.pose           = pose(:,1:round(56*100));
        raw_data{ii}.ft             = ft(:,1:round(56*100));
        raw_data{ii}.joint_states   = joint_states(:,1:round(56*143));
        
    else
        raw_data{ii}.pose           = pose(:,1:round(80*100));
        raw_data{ii}.ft             = ft(:,1:round(80*100));
        raw_data{ii}.joint_states   = joint_states(:,1:round(80*143));
    end
    
    % Bag Times
    t_begin = raw_data{ii}.pose(end,1); t_end = raw_data{ii}.pose(end,end); 
    
    raw_data{ii}.dt             = pose(8,end) - pose(8,end-1);
    raw_data{ii}.times          = [t_begin t_end];
    raw_data{ii}.name           = strrep(bags(ii).name,'.bag','');    
    
end

%% Visualize EE positions and FT readings
figure('Color',[1 1 1])
N = length(bags);
colors = hsv(N);
for ii=1:N
chosen_demo  = ii; 
sample_size  = 2;

% Extract desired trajectory
recording = raw_data{chosen_demo};

% Extract desired variables
ee_traj    = recording.pose(1:3,1:sample_size:end);

% Plot EE Trajectories
scatter3(ee_traj(1,:), ee_traj(2,:), ee_traj(3,:), 7.5, 'MarkerEdgeColor','k','MarkerFaceColor',colors(ii,:)); hold on;
end
xlabel('$\xi_1$', 'Interpreter', 'LaTex', 'FontSize',15);
ylabel('$\xi_2$', 'Interpreter', 'LaTex','FontSize',15);
zlabel('$\xi_3$', 'Interpreter', 'LaTex','FontSize',15);
grid on
title('Cartesian EE Position Trajectories',  'Interpreter', 'LaTex','FontSize',15)
axis equal


%% Pre-process data to match Acquisition Frequencies
for ii=1:N
    options.match_hz    = 1;
    options.smooth_fact = 0.025;
    options.do_plot     = 1;
    options.tool_frame  = eye(4);
    options.base_frame  = [];
    options.title_name  = ['Sequence: ',raw_data{ii}.name];
    [X, H, J, t] = preProcessEEDataJoints(raw_data{ii}.pose, raw_data{ii}.ft, raw_data{ii}.times, ...
        raw_data{ii}.joint_states, options);
    
    % Gather new semi-processed data 
    demo_data{ii}.X = X;
    demo_data{ii}.H = H;
    demo_data{ii}.J = J;
    demo_data{ii}.t = t;
    demo_data{ii}.name = raw_data{ii}.name;
    demo_data{ii}.dt   = raw_data{ii}.dt;
end

%% Save data to matfile
matfile = strcat(data_dir,'demos_combined_data.mat');
save(matfile,'raw_data','demo_data','bags','bag_dir')
