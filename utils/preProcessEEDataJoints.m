function [ X, H, Joints, t ] = preProcessEEDataJoints(pose, ft, times, joints, options)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here

% Extract variables of different type
pos   = pose(1:3,:); ori    = pose(4:7,:); pose_t = pose(8,:);
force = ft(1:3,:);   torque = ft(4:6,:);   ft_t   = ft(7,:);

begin_time = times(1); end_time = times(2);

% Compute some useful pre-process variables
duration  = end_time - begin_time;  
pose_hz   = round(size(pos,2)/duration);
ft_hz     = round(size(force,2)/duration);
joint_hz  = round(size(joints,2)/duration);

% Set Sub-sampling rate
if (pose_hz == ft_hz || options.match_hz == 0), sub = 1; else sub = pose_hz/ft_hz;end
if (joint_hz == pose_hz || options.match_hz == 0), sub_j = 1; else sub_j = joint_hz/pose_hz;end

% Fix Rotation Discontinuities in Rotation
ori_fx = checkRotations(ori);

% Match Framerate between pose and ft
Joints = [];
if options.match_hz == 1 
    pose_sub_t = pose_t(:,1:sub:end); pose_sub_t = pose_sub_t(:,1:size(force,2));
    pos_sub    = pos(:,1:sub:end);    pos_sub    = pos_sub(:,1:size(force,2));
    ori_sub    = ori_fx(:,1:sub:end); ori_sub    = ori_sub(:,1:size(force,2));
    for i=1:size(joints,1)
        Joints(i,:) = resample(joints(i,:),pose_hz,joint_hz);
    end
else
    pose_sub_t = pose_t;
    pos_sub    = pos;
    ori_sub    = ori_fx;
end


% Concatenate Pose
X_p = [pos_sub;ori_sub];

% Convert to Homogeneous MatriX Representation
H = convert2H(X_p);

% If base frame given apply translation
if ~isempty(options.base_frame)    
    % Transform with base reference frame 
    for kk=1:length(X_p), H(1:3,4,kk) = (H(1:3,4,kk) + options.base_frame(1:3,4)); end        
end

% Add Tool Transform
for kk=1:length(X_p), H(:,:,kk) = H(:,:,kk)*options.tool_frame; end    

% Convert back 7D Representation
X_p(1:7,:) = convert2X(H);    

% Check Orientatons again
X_p(4:7, :) = checkRotations(X_p(4:7, :));

% Smooth FT Data    
sm_ft = [force;torque];
for kk=1:size(sm_ft,1)
    sm_ft(kk,:) = smooth(sm_ft(kk,:),options.smooth_fact,'moving');
end
force = sm_ft(1:3,:); torque = sm_ft(4:6,:);

% Rotate Force and Torques
for kk=1:length(force), force(1:3,kk) = (options.tool_frame(1:3,1:3)*force(1:3,kk)); torque(1:3,kk) = (options.tool_frame(1:3,1:3)*torque(1:3,kk)); end    

% Processed Data
X = [X_p; force; torque];
Joints = Joints(:,1:length(X));

% Time Stamp
t = [ft_t];

% Plot Data from Bag
if options.do_plot == 1    
    plotEEData( X, t ,options.title_name)
end

end

