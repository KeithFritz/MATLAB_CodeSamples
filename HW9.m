% Keith Fritz
% MXET 400-500
% Homework #9
% April 20th 2021
clear;clc;

% Problem 1:
% Generates matrix a to be used to generate trajectory cubic polynomials
t1 = 2;
t2 = 5;
a1 = traj(0, t1, 0, 45, 0, 15); % From t=0 to t=t1
a2 = traj(t1, t2, 45, 135, 15, 0); % From t=t1 to t=t2

% Generates q(t), dq(t), and ddq(t) trajectory polynomials from t=0 to t=t1
% (1) and from t=t1 to t=t2 (2)
syms t
q1 = a1(1) + a1(2)*t + a1(3)*t^2 + a1(4)*t^3
dq1 = diff(q1, t)
ddq1 = diff(dq1, t)

q2 = a2(1) + a2(2)*t + a2(3)*t^2 + a2(4)*t^3
dq2 = diff(q2, t)
ddq2 = diff(dq2, t)


% Problem 2:
% Plots the results in three graphs
% Creates time array for each movement
t1 = [0:0.01:2];
t2 = [2:0.01:5];

% Graphs plot from 0 to t1 using set of polynomials (1),
% and from t1 to t2 using set of polynomials (2).

% q(t) Plot
figure(1) % q(t) vs t
plot(t1, subs(q1,t1))
hold on
plot(t2, subs(q2, t2))
hold off
xlabel('Time(sec)')
ylabel('Angle(deg)')
title('Plot of q(t) vs t')

% dq(t) Plot
figure(2) % dq(t) vs t
plot(t1, subs(dq1,t1))
hold on
plot(t2, subs(dq2, t2))
hold off
xlabel('Time(sec)')
ylabel('Angle(deg)')
title('Plot of dq(t) vs t')

% ddq(t) Plot
figure(3) % ddq(t) vs t
plot(t1, subs(ddq1,t1))
hold on
plot(t2, subs(ddq2, t2))
hold off
xlabel('Time(sec)')
ylabel('Angle(deg)')
title('Plot of ddq(t) vs t')


% Function to generate trajectory matrix 'a'
function a = traj(t0, tf, q0, qf, v0, vf)

A = [1, t0, t0^2, t0^3; 0, 1, 2*t0, 3*t0^2; 1, tf, tf^2, tf^3; 0, 1, 2*tf, 3*tf^2];
b = [q0, v0, qf, vf]';
a = A\b;
end