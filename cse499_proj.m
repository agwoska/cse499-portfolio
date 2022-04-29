% @file cse499_proj.m
% @author Andrew Woska, Daniel Maas, Imani Muhammed-Graham
% @date 2022-03-14
% for CSE 499 - Embedded Controls
% Description
%   This file is used to calculate and test the PID controller
%   for a self-balancing robot

clc
clear
close all


M = 0.2;
m = 0.1;
b = 0.1;
I = 0.006;
g = 9.8;
l = 0.2;
q = (M+m)*(I+m*l^2)-(m*l)^2;

num = [m*l/q, 0];
denom = [ 1, b*(I+m*l^2)/q, -(M+m)*m*g*l/q, -b*m*g*l/q ];

% s = .001; % sample timing
% PID values
Kp = 10;
Ki = 7;
Kd = .75;
% controller
sys = tf(num,denom)
c = pid(Kp,Ki,Kd)
t = feedback(sys,c)
% create figures
figure
pzmap(t)
figure
step(t)
