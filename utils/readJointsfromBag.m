function [ joint_states, joint_t ] = readJointsfromBag(bag, joint_state_topic)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

% Functions for each topic type
joint_pos = @(j) j.position(1:7,:);

% Read Pose Topics
[msgs, meta]      = bag.readAll(joint_state_topic);
[joint_states]   = ros.msgs2mat(msgs, joint_pos);
joint_t = cellfun(@(x) x.time.time, meta); % Time Stamps


end

