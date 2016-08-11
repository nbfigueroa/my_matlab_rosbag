function [ ] = plotEEData( X, t , titlename)
%UNTITLED8 Summary of this function goes here
%   Detailed explanation goes here

begin_time = t(1);

figure('Color', [1 1 1]);
subplot(4,1,1);
plot(t - begin_time, X(1:3,:)','--');
legend('x','y','z')

subplot(4,1,2);
plot(t - begin_time, X(4:7,:)','-');
legend('q_w','q_i', 'q_j', 'q_k')

subplot(4,1,3);
plot(t - begin_time, X(8:10,:)','-');
legend('f_x','f_y', 'f_z')

subplot(4,1,4);
plot(t - begin_time, X(11:end,:)','-');
legend('\tau_x','\tau_y', '\tau_z')

suptitle(titlename)

end

