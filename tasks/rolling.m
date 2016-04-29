clear rosbag_wrapper;
clear ros.Bag;
clear all

des_pose_topic = '/cart_to_joint/des_ee_pose';
des_ft_topic = '/cart_to_joint/des_ee_ft';
est_pose_topic = '/joint_to_cart/est_ee_pose';
est_ft_topic = '/joint_to_cart/est_ee_ft';
joint_topic = '/joint_states';

acc_pos = @(p) p.pose.position;
acc_force = @(w) w.wrench.force;
acc_jvel = @(x) x.velocity;
for i=1:9
    disp(['Reading ' int2str(i)]);
bag = ros.Bag.load(['../' num2str(i) '.bag']);

[msgs, meta] = bag.readAll(des_pose_topic);
[pos_des] = ros.msgs2mat(msgs, acc_pos); % Convert struct to 3-by-N matrix of linear velcoity
pos_des_t = cellfun(@(x) x.time.time, meta); % Get timestamps

[msgs, meta] = bag.readAll(est_pose_topic);
[pos_est] = ros.msgs2mat(msgs, acc_pos); % Convert struct to 3-by-N matrix of linear velcoity
pos_est_t = cellfun(@(x) x.time.time, meta); % Get timestamps

[msgs, meta] = bag.readAll(des_ft_topic);
[ft_des] = ros.msgs2mat(msgs, acc_force); % Convert struct to 3-by-N matrix of linear velcoity
ft_des_t = cellfun(@(x) x.time.time, meta); % Get timestamps

[msgs, meta] = bag.readAll(est_ft_topic);
[ft_est] = ros.msgs2mat(msgs, acc_force); % Convert struct to 3-by-N matrix of linear velcoity
ft_est_t = cellfun(@(x) x.time.time, meta); % Get timestamps

[msgs, meta] = bag.readAll(joint_topic);
[vel] = ros.msgs2mat(msgs, acc_jvel); % Convert struct to 3-by-N matrix of linear velcoity
vel_t = cellfun(@(x) x.time.time, meta); % Get timestamps

disp('Done');
figure(1); clf;
subplot(2,1,1);cla; hold on
plot(pos_des_t, pos_des','--');
hold on
plot(pos_est_t, pos_est','-');

subplot(2,1,2);cla; hold on
plot(ft_des_t, ft_des(3,:)','--');
hold on
plot(ft_est_t, ft_est(3,:)','-');

% subplot(3,1,3);cla; hold on
% plot(vel_t, vel');
pause;
end

%% Read all messages on a few topics

msgs = bag.readAll({topic1, topic2});

fprintf('Read %i messages\n', length(msgs));

%% Re-read msgs on topic1 and get their metadata
[msgs, meta] = bag.readAll(topic1);
fprintf('Got %i messages, first one at time %f\n', ...
    length(msgs), meta{1}.time.time);

%% Read messages incrementally
bag.resetView(topic1);
count = 0;
while bag.hasNext();
    [msg, meta] = bag.read();
    count = count + 1;
end

%% See definitions of messages contained within the bag
twist_definition = bag.definition('geometry_msgs/Twist')

% Setting 'raw' to true shows the original message definition with comments
raw = true;
raw_twist_definition = bag.definition('geometry_msgs/Twist', raw)

% When it's unambiguous, you can drop the package name.  You can also get
% definitions for messages defined in other messages;
% geometry_msgs/Quaternion comes from geometry_msgs/Twist
quaternion_definition = bag.definition('Quaternion', true)

%% Convert velocity messages to a matrix to plot linear speed
[msgs, meta] = bag.readAll(topic1); % Messages are structs
accessor = @(twist) twist.linear;
[xyz] = ros.msgs2mat(msgs, accessor); % Convert struct to 3-by-N matrix of linear velcoity
times = cellfun(@(x) x.time.time, meta); % Get timestamps
% Plot linear speed over time
plot(times, xyz(1, :));
ylim([-2.5 2.5]);

%% Learn more
doc ros.Bag
