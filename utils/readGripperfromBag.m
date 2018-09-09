function [ Gripper ] = readGripperfromBag(bag, gripper_topic, ee_pose_msgs)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

% Functions for each topic type
gripper  = @(grip) grip(8,:);

% Read Pose Topics
[msgs, meta]    = bag.readAll(gripper_topic);
gripper_states = ros.msgs2mat(msgs, gripper); 
gripper_phases = cellfun(@(x) x.time.time, meta); % Time Stamps

Gripper = [gripper_states; gripper_phases];

Gripper = zeros(2,size(ee_pose_msgs,2));
Gripper(2,:) = ee_pose_msgs(end,:);
for i=1:(size(gripper_phases, 2))    
    if i <  size(gripper_phases, 2)
            phases    = [gripper_phases(i) gripper_phases(i+1)];
            Idx_gripper = knnsearch(Gripper(2,:)', phases');
    else
            Idx_gripper = knnsearch(Gripper(2,:)', gripper_phases(i));
            Idx_gripper = [Idx_gripper; size(gripper_phases, 2)];
    end
    
    Gripper(1,Idx_gripper(1):Idx_gripper(2)-1) = gripper_states(i)*ones(1,Idx_gripper(2)-Idx_gripper(1));
    
    if i == 1
        Gripper(1,1:Idx_gripper(1)-1) = 255*ones(1,Idx_gripper(1)-1);
    end
end


end

