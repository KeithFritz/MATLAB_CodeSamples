clc;clear;
%Keith Fritz - Lab 5
%ESET 462

%Physical Constants:
%{
% #Example
K = 5;      %gain
R = 2;    %Ohms
L = 0.5;    %Henrys
Km = 0.1;   %torque constant
Kb = 0.1;   %back emf constant
Kf = 0.2;   %Nms
J = 0.02;   %kg.m^2/s^2
%}
%{
% #1
K = 5;      %gain
R = 0.1;    %Ohms
L = 0.05;    %Henrys
Km = 1;   %torque constant
Kb = 1;   %back emf constant
Kf = 2;   %Nms
J = 0.2;   %kg.m^2/s^2
%}
%%{
% #2
K = 5;      %gain
R = 0.01;    %Ohms
L = 0.001;    %Henrys
Km = 5;   %torque constant
Kb = 5;   %back emf constant
Kf = 5;   %Nms
J = 1;   %kg.m^2/s^2
%%}

h1 = tf(Km, [L R]); %armature
h2 = tf(1, [J Kf]); %eqn of mption

dcm = h1*h2; %w = h2 * (h1*Va +Td)
dcm = feedback(dcm,Kb); %close back emf loop

figure(1) % DC Motor speed controller velocity vs time
stepplot(dcm*100);

figure(2) % Root locus plot
h = rlocusplot(tf(1, [1 0]) * dcm(1));
setoptions(h, 'FreqUnits', 'rad/s');
xlim([-15 5]);
ylim([-15 15]);

C = tf(K, [1 0]); %compensator K/s

cl_rloc = feedback(dcm * C,1);
figure(3)
stepplot(cl_rloc*100);
title('DC Electric Motor Speed Controller')
ylabel("Angular Velocity (rad/s)")
xlabel("Time")
