clc; clear;
% ESET 462 - Lab 6
% Keith Fritz

% Define initial conditions
% #1:
Kp = 1; Kd = 0; Ki = 50;

% #2:
%Kp = 1; Kd = 50; Ki = 0;

% #3:
%Kp = 1; Kd = 5; Ki = 5;

% Critically Damped PID coefficients:
% (Determined through manipuation of PID Tuner)
%Kp = 0; Kd = 0; Ki = 3.5;

R = 1;
C = 0.1;
t = 0:0.001:50;

s = tf('s');
pid = Kp + Ki/s + Kd*s;
%pid = pid(Kp,Ki,Kd)

Vc = 1/(R*C*s + 1);
T = feedback(Vc.*pid,1);

subplot(2,1,1)
step(Vc,t)
title("Open Loop System")
ylabel("Voltage")

subplot(2,1,2)
step(T,t)
title("Closed Loop System")
ylabel("Voltage")

pidTuner(Vc,pid)