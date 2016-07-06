%/**
% * @file log_plot.m
% *
% * log plot
% *
% * @author Ricardo <tsao.ricardo@iac.com.tw>
% */

clear all;
clc;

log_name = textread('log.txt', '%s');
log_name = log_name{1};

log = csvread(log_name);
log = log';

t = log(1,:) / 1000;

sv_vx = log( 2,:);
cv_vx = log( 3,:);
pv_vx = log( 4,:);

sv_vy = log( 5,:);
cv_vy = log( 6,:);
pv_vy = log( 7,:);

sv_w0 = log( 8,:);
cv_w0 = log( 9,:);
pv_w0 = log(10,:);

figure
plot(t, sv_vx, t, cv_vx, t, pv_vx);
grid on;

title('Velocity Response X');
legend('sv.vx', 'cv.vx', 'pv.vx');
xlabel('Time (sec)');
ylabel('Velocity (m/s)');

figure
plot(t, sv_vy, t, cv_vy, t, pv_vy);
grid on;

title('Velocity Response Y');
legend('sv.vy', 'cv.vy', 'pv.vy');
xlabel('Time (sec)');
ylabel('Velocity (m/s)');

figure
plot(t, sv_w0, t, cv_w0, t, pv_w0);
grid on;

title('Angular Velocity Response W0');
legend('sv.w0', 'cv.w0', 'pv.w0');
xlabel('Time (sec)');
ylabel('Velocity (m/s)');
