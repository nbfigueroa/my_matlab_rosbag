%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Demo script used to extract trajectories of two motion primitives %%
%% using the gripper state as the phase indicator (segmenter)        %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Load Dataset and Compute Velocities
clear all; clc; close all

% Load Data from Mat File
mat_data_dir = '../../jmlr-2019/mat/';

%% To load Rim Wiping Data
matfile = strcat(mat_data_dir,'demos_individual_data.mat');
mat_data = load(matfile);
for i=1:5
    demo_data{i} = mat_data.demo_data{i};
end
clear matfile
matfile = strcat(mat_data_dir,'demos_combined_data.mat');
mat_data = load(matfile);
demo_data{length(demo_data)+1} = mat_data.demo_data{1};

%% To load Fender Wiping Data
matfile = strcat(mat_data_dir,'demos_individual_data.mat');
mat_data = load(matfile);
for i=1:2
    demo_data{i} = mat_data.demo_data{5+i};
end
clear matfile
matfile = strcat(mat_data_dir,'demos_combined_data.mat');
mat_data = load(matfile);
demo_data{length(demo_data)+1} = mat_data.demo_data{2};

%% Extract trajectories per primitive - computing velocities and smoothing
clc;
data_primitive_1 = [];
data_primitive_0   = [];
p = 0; o= 0;
vel_cutting = 0.01;
pos_cutting = 1e-3;
for n=N:N
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
            % Do filtering
            dx_nth = sgolay_time_derivatives(Xi_ref(:,segment(s,1):segment(s,2))', dt, 1, 2, 21);
            Xi_ref_tmp     = dx_nth(:,:,1)';
            Xi_dot_ref_tmp = dx_nth(:,:,2)';                           

            % trimming demonstrations (Removing measurements with zero velocity)
            zero_vel_idx = find(vecnorm(Xi_dot_ref_tmp) < 5e-4);
            fprintf ('Primitive 1: %d datapoints removed ... ', length(zero_vel_idx));
            Xi_ref_tmp(:,zero_vel_idx) = [];
            Xi_dot_ref_tmp(:,zero_vel_idx) = [];                
            
            % Do filtering again
            dx_nth = sgolay_time_derivatives(Xi_ref_tmp', dt, 1, 2, 15);
            Xi_ref_tmp     = dx_nth(:,:,1)';
            Xi_dot_ref_tmp = dx_nth(:,:,2)'; 
            
            % Check final measurements                        
            [idx_end, dist_end] = knnsearch( Xi_ref_tmp(:,end-round(1/dt):end)', Xi_ref_tmp(:,end)', 'k',round(1/dt));
            idx_zeros = idx_end(dist_end < pos_cutting);
            Xi_ref_tmp(:,idx_zeros)=[];
            Xi_dot_ref_tmp(:,idx_zeros)=[];
            
            % Make last measurment 0 velocity and scale the previous
            Xi_dot_ref_tmp(:,end)   = zeros(size(Xi_ref_tmp,1),1);
            for k =1:20
                Xi_dot_ref_tmp(:,end-k) = (Xi_dot_ref_tmp(:,end-k) +  Xi_dot_ref_tmp(:,end-(k-1)))/2;
            end
            
            % feed to data structure
            data_primitive_1{p,1} = [Xi_ref_tmp; Xi_dot_ref_tmp];
        else
            o = o + 1;
            % Do filtering
            dx_nth = sgolay_time_derivatives(Xi_ref(:,segment(s,1):segment(s,2))', dt, 1, 2, 21);
            Xi_ref_tmp     = dx_nth(:,:,1)';
            Xi_dot_ref_tmp = dx_nth(:,:,2)';
            
            % trimming demonstrations (Removing measurements with zero velocity)
            zero_vel_idx = find(vecnorm(Xi_dot_ref_tmp) < 1e-3);
            fprintf ('Primitive 0: %d datapoints removed \n', length(zero_vel_idx));
            Xi_ref_tmp(:,zero_vel_idx) = [];
            Xi_dot_ref_tmp(:,zero_vel_idx) = [];
            
            if length(Xi_dot_ref_tmp) > round(1/dt)
                % Check final measurements
                [idx_end, dist_end] = knnsearch( Xi_ref_tmp(:,end-round(1/dt):end)', Xi_ref_tmp(:,end)', 'k', round(1/dt));
                idx_zeros = idx_end(dist_end < pos_cutting);
                Xi_ref_tmp(:,idx_zeros)=[];
                Xi_dot_ref_tmp(:,idx_zeros)=[];
                
                % Make last measurment 0 velocity and scale the previous
                Xi_dot_ref_tmp(:,end)   = zeros(size(Xi_ref_tmp,1),1);
                for k =1:20
                    Xi_dot_ref_tmp(:,end-k) = (Xi_dot_ref_tmp(:,end-k) +  Xi_dot_ref_tmp(:,end-(k-1)))/2;
                end
            else
                Xi_ref_tmp = [];
                Xi_dot_ref_tmp = [];
            end
          
            % feed to data structure
            data_primitive_0{o,1} = [Xi_ref_tmp; Xi_dot_ref_tmp];
        end
    end        
end


% Remove for empty entries if any
data_primitive_1 = data_primitive_1(~cellfun('isempty',data_primitive_1)); 
data_primitive_0 = data_primitive_0(~cellfun('isempty',data_primitive_0)); 

%% Visualize EE position and Velocities for Non-linear Motion Primitive
figure('Color',[1 1 1])
subplot(1,2,1);
% Plot EE Trajectories
for p=1:length(data_primitive_1)
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
for p=1:length(data_primitive_0)
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
for p=1:2:length(data_primitive_0)
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
matfile = strcat(mat_data_dir,'demos_Sept29_12pm_processed_data.mat');
% save(matfile,'data_primitive_1','data_primitive_0','dt')
save(matfile,'data_primitive_0','dt')
