clear rosbag_wrapper;
clear ros.Bag;
clear all

des_ft_topic = '/cart_to_joint/des_ee_ft';
est_ft_topic = '/joint_to_cart/est_ee_ft';

file_pref = 'stepforce';
file_ind={'5', '10', '15', '20'};

% file_pref = 'oscforce';
% file_ind={'5_2', '5_5', '5_10', '10_1','10_2','10_5'};

acc_force = @(w) w.wrench.force;
for i=1:length(file_ind)
    fname = ['../force_test_data/' file_pref file_ind{i} '.bag'];
    disp(['Reading ' fname]);
bag = ros.Bag.load(fname);

[msgs, meta] = bag.readAll(des_ft_topic);
[ft_des] = ros.msgs2mat(msgs, acc_force); % Convert struct to 3-by-N matrix of linear velcoity
ft_des_t = cellfun(@(x) x.time.time, meta); % Get timestamps

[msgs, meta] = bag.readAll(est_ft_topic);
[ft_est] = ros.msgs2mat(msgs, acc_force); % Convert struct to 3-by-N matrix of linear velcoity
ft_est_t = cellfun(@(x) x.time.time, meta); % Get timestamps

ft_des_t = ft_des_t-ft_est_t(1);
ft_est_t = ft_est_t-ft_est_t(1);
disp('Done');

des=[];
next_ind = 0; des_val = [0;0;0];

for j=1:size(ft_est_t,2)
    curr_t = ft_est_t(j);
    if next_ind ~= length(ft_des_t) && curr_t > ft_des_t(next_ind+1)
        next_ind = next_ind+1;
        des_val  = ft_des(:, next_ind);
    end
    des = [des, des_val];
end
figure(1); 
cla; hold on; box on; grid on
plot(ft_est_t, des','--','Linewidth',2);
hold on
plot(ft_est_t, ft_est','-');

legend('x','y','z');
title([file_pref ' ' file_ind{i}],'interpreter','none');

pause;
end

