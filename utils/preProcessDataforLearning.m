function [ X, t ] = preProcessDataforLearning(pose, ft, bag, options)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here


pos   = pose(1:3,:); ori    = pose(4:7,:); pose_t = pose(8,:);
force = ft(1:3,:);   torque = ft(4:6,:);   ft_t   = ft(7,:);

% Compute some useful pre-process variables
duration = bag.time_end - bag.time_begin;  
pose_hz  = round(size(pos,2)/duration);
ft_hz    = round(size(force,2)/duration);

% Set Sub-sampling rate
if (pose_hz == ft_hz || options.match_hz == 0), sub = 1; else sub = pose_hz/ft_hz;end

% Fix Rotation Discontinuities in Rotation
ori_fx = checkRotations(ori);

% Match Framerate between pose and ft
if options.match_hz == 1 
    pose_sub_t = pose_t(:,1:sub:end); pose_sub_t = pose_sub_t(:,1:size(force,2));
    pos_sub    = pos(:,1:sub:end);    pos_sub    = pos_sub(:,1:size(force,2));
    ori_sub    = ori_fx(:,1:sub:end); ori_sub    = ori_sub(:,1:size(force,2));
else
    pose_sub_t = pose_t;
    pos_sub    = pos;
    ori_sub    = ori_fx;
end



% Smooth FT Data    
sm_ft = [force;torque];
for kk=1:size(sm_ft,1)
    sm_ft(kk,:) = smooth(sm_ft(kk,:),options.smooth_fact,'moving');
end
force = sm_ft(1:3,:); torque = sm_ft(4:6,:);

% Processed Data
X = [pos_sub; ori_sub; force; torque];
% Time Stamp
t = [ft_t];

% Plot Data from Bag
if options.do_plot == 1
    figure('Color', [1 1 1]);
    subplot(4,1,1);
    plot(pose_sub_t - bag.time_begin, pos_sub','--');
    legend('x','y','z')

    subplot(4,1,2);
    plot(pose_sub_t - bag.time_begin, ori_sub','-');
    legend('q_w','q_i', 'q_j', 'q_k')

    subplot(4,1,3);
    plot(ft_t - bag.time_begin, force','-');
    legend('f_x','f_y', 'f_z')

    subplot(4,1,4);
    plot(ft_t - bag.time_begin, torque','-');
    legend('\tau_x','\tau_y', '\tau_z')

    suptitle(options.title_name)
end

end

