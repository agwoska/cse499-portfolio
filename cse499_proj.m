% @file cse499_proj.m
% @author Andrew Woska, Daniel Maas, Imani Muhammed-Graham
% @date 2022-03-14
% for CSE 499 - Embedded Controls
% Description
%   This file is used to calculate and test the PID controller
%   for a self-balancing robot
%
% Formulas
% 

clc
clear
close all


M = 0.4;
m = 0.3;
b = 0.1;
I = 0.006;
g = 9.8;
l = 0.3;
q = (M+m)*(I+m*l^2)-(m*l)^2;

num = [m*l/q, 0];
denom = [ 1, b*(I+m*l^2)/q, -(M+m)*m*g*l/q, -b*m*g*l/q ];

s = .001

Kp = 19;
Ki = 10;
Kd = 5;

sys = tf(num,denom)
% csys = c2d(sys,s)
c = pid(Kp,Ki,Kd)
% c1 = pid(Kp,Ki,Kd,s)
t = feedback(sys,c)
% t1 = feedback(csys,c1)

figure
pzmap(t)
% 
% figure
% rlocus(t)

% figure
% nyquist(t)

figure
step(t)
% axis([0, 5, -0.2, 0.2]);