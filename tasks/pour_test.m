clear rosbag_wrapper;
clear ros.Bag;
clear all

est_pose_topic = '/joint_to_cart/est_ee_pose';
est_ft_topic = '/joint_to_cart/est_ee_ft';

file_pref = '~/data/devel/catkin_ws_robohow/src/rtkhydro/pouring_data/';
a=dir([file_pref '*.bag']);

acc_force = @(w) w.wrench.force;
acc_pos = @(w) w.pose.orientation;
converter = @(x) (R2rpy(quaternion2matrix([x(4);x(1:3)]))');
for i=1:length(a)
    fname = a(i).name;
    disp(['Reading ' fname]);
bag = ros.Bag.load([file_pref fname]);


[msgs, meta] = bag.readAll(est_ft_topic);
[ft_est] = ros.msgs2mat(msgs, acc_force); % Convert struct to 3-by-N matrix of linear velcoity
ft_est_t = cellfun(@(x) x.time.time, meta); % Get timestamps

[msgs, meta] = bag.readAll(est_pose_topic);
[pos_est] = ros.msgs2mat(msgs, acc_pos, converter); % Convert struct to 3-by-N matrix of linear velcoity
pos_est_t = cellfun(@(x) x.time.time, meta); % Get timestamps

ind = find(pos_est(1,:) > 0);
pos_est(1,ind) = pos_est(1,ind)-2*pi;
pos_est_t = pos_est_t-pos_est_t(1);
ft_est_t = ft_est_t-ft_est_t(1);
disp('Done');

figure(1); 
subplot(2,1,1); cla; hold on; box on; grid on
plot(ft_est_t, ft_est','--','Linewidth',2);
legend('x','y','z');
subplot(2,1,2); cla; hold on; box on; grid on
plot(pos_est_t, 180/pi*pos_est(1,:)','--','Linewidth',2);




pause;
end

