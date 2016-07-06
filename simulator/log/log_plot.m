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

n = length(log(1,:));
t = log(1,:) / 1000;

sv = log(2,:); % vx.sv
cv = log(3,:); % vx.cv
pv = log(4,:); % vx.pv

figure
plot(t, sv, t, cv, t, pv);
grid on;

title('Velocity Response');
legend('vx.sv', 'vx.cv', 'vx.pv');
xlabel('Time (sec)');
ylabel('Velocity (m/s)');
