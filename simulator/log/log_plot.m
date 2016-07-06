%/**
% * @file log_plot.m
% *
% * log plot
% *
% * @author Ricardo <tsao.ricardo@iac.com.tw>
% */

clear all;
clc;
log = csvread('log.csv');
log = log';

t = 120/1000; % 120 ms
n = length(log(1,:));
t = (0:(n - 1)) * (t);

sv = log(1,:); % vx.sv
cv = log(2,:); % vx.cv
pv = log(3,:); % vx.pv

figure
plot(t, sv, t, cv, t, pv);
grid on;

title('Velocity Response');
legend('vx.sv', 'vx.cv', 'vx.pv');
xlabel('Time (sec)');
ylabel('Velocity (m/s)');
