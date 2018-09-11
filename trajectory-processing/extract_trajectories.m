%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Demo script used to extract trajectories of two motion primitives %%
%% using the gripper state as the phase indicator (segmenter)        %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Load Dataset and Compute Velocities
clear all; clc; close all

% Load Data from Mat File
mat_data_dir = '../../corl-2018/mat/';
matfile = strcat(mat_data_dir,'demos_Scenario2_Sept10_00am_raw_data.mat');
load(matfile);

% Extract EE Positions and Remove zero velocity points (trim)
dt = data{1}.dt; N = length(data);
data_ = []; Xi_ref = []; tol_cutting = 0.5;
for n=1:N        
    % Load into data structure    
    data_{n}.Xi_ref     = data{n}.ee_pose(1:3,:);
    data_{n}.label      = data{n}.gripper_state(1,:)/255;
end
clear data; data = data_; clear data_

%% Extract trajectories per primitive - computing velocities and smoothing
data_primitive_1 = [];
data_primitive_0   = [];
p = 0; o= 0;
for n=1:N
    labels = data{1,n}.label;
    change_points = find(ischange(labels) == 1);
    clear segment label
    k = 0;
    for c=1:length(change_points)+1
        k = k + 1;
        if c == 1
            segment(k,1:2) = [1 change_points(1,c)-1];            
            label = labels(1);
        elseif c == (length(change_points)+1)
            segment(k,1:2) = [change_points(1,c-1)+1 length(labels)];
            label = labels(change_points(1,c-1)+1);
        else
            segment(k,1:2) = [change_points(1,c-1)+1 change_points(1,c)-1];            
            label = labels(change_points(1,c-1)+1);
        end        
        segment(k,3) = label;
    end
    Xi_ref     = data{1,n}.Xi_ref;   
    for s=1:length(segment)
        if segment(s,3) == 1
              p = p + 1;
              dx_nth = sgolay_time_derivatives(Xi_ref(:,segment(s,1):segment(s,2))', dt, 1, 2, 21);
              Xi_ref_tmp     = dx_nth(:,:,1)';
              Xi_dot_ref_tmp = dx_nth(:,:,2)';              
              data_primitive_1{p,1} = [Xi_ref_tmp; Xi_dot_ref_tmp];
        else
              o = o + 1;
              dx_nth = sgolay_time_derivatives(Xi_ref(:,segment(s,1):segment(s,2))', dt, 1, 2, 21);
              Xi_ref_tmp     = dx_nth(:,:,1)';
              Xi_dot_ref_tmp = dx_nth(:,:,2)';              
              data_primitive_0{o,1} = [Xi_ref_tmp; Xi_dot_ref_tmp];
        end
    end
        
end
%% Visualize EE position and Velocities for Non-linear Motion Primitive
figure('Color',[1 1 1])
subplot(1,2,1);
% Plot EE Trajectories
for p=1:2:length(data_primitive_1)
    ee_pos = data_primitive_1{p}(1:3,:);
    plot3(ee_pos(1,:)',ee_pos(2,:)',ee_pos(3,:)', 'Color',[0 rand rand], 'LineWidth',1.5); hold on;
end

xlabel('$\xi_1$', 'Interpreter', 'LaTex', 'FontSize',15);
ylabel('$\xi_2$', 'Interpreter', 'LaTex','FontSize',15);
zlabel('$\xi_3$', 'Interpreter', 'LaTex','FontSize',15);
grid on
title('Non-linear Primitive - Cartesian EE Position Trajectories',  'Interpreter', 'LaTex','FontSize',15)

subplot(1,2,2);
% Plot EE Velocity Profiles
for p=1:2:length(data_primitive_1)
    ee_vel = data_primitive_1{p}(4:6,:);
    plot3(ee_vel(1,:)',ee_vel(2,:)',ee_vel(3,:)', 'Color',[0 rand rand], 'LineWidth',1.5); hold on;
end
xlabel('$\xi_1$', 'Interpreter', 'LaTex', 'FontSize',15);
ylabel('$\xi_2$', 'Interpreter', 'LaTex','FontSize',15);
zlabel('$\xi_3$', 'Interpreter', 'LaTex','FontSize',15);
grid on
title('Non-linear Primitive - Cartesian EE Velocity Profiles',  'Interpreter', 'LaTex','FontSize',15)


%% Visualize EE position and Velocities for Picking (Back) Motion Primitive
figure('Color',[1 1 1])
% Plot EE Trajectories
subplot(1,2,1);
for p=1:2:length(data_primitive_0)
    ee_pos = data_primitive_0{p}(1:3,:);
    c = rand/3;
    plot3(ee_pos(1,:)',ee_pos(2,:)',ee_pos(3,:)', 'Color',[0.5+c 0.5+c 0.5+c], 'LineWidth',1.5); hold on;
end

xlabel('$\xi_1$', 'Interpreter', 'LaTex', 'FontSize',15);
ylabel('$\xi_2$', 'Interpreter', 'LaTex','FontSize',15);
zlabel('$\xi_3$', 'Interpreter', 'LaTex','FontSize',15);
grid on
title('Picking Primitive - Cartesian EE Position Trajectories',  'Interpreter', 'LaTex','FontSize',15)

subplot(1,2,2);
% Plot EE Velocity Profiles
for p=1:2:length(data_primitive_1)
    ee_vel = data_primitive_0{p}(4:6,:);
    c = rand/3;
    plot3(ee_vel(1,:)',ee_vel(2,:)',ee_vel(3,:)', 'Color',[0.5+c 0.5+c 0.5+c], 'LineWidth',1.5); hold on;
end
xlabel('$\xi_1$', 'Interpreter', 'LaTex', 'FontSize',15);
ylabel('$\xi_2$', 'Interpreter', 'LaTex','FontSize',15);
zlabel('$\xi_3$', 'Interpreter', 'LaTex','FontSize',15);
grid on
title('Picking Primitive -  Cartesian EE Velocity Profiles',  'Interpreter', 'LaTex','FontSize',15)


%% Save processed data to matfile
matfile = strcat(mat_data_dir,'demos_Scenario2_Sept10_00am_processed_data.mat');
save(matfile,'data_primitive_1','data_primitive_0','dt')

