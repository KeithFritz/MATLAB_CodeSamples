% Keith Fritz
% MXET 400-500
% Homework #10 (PID)
% April 27th 2021
clear;clc;

% Single-Joint Robot (HW 10) [PID Controller]

M = 0.5;    % scalar inertia of the link
m = 1;      % mass of the link
r = 0.1;    % distance from the axis to the center of mass of the link
g = 9.81;   % gravitational acceleration
b = 0.1 ;   % friction coefficient 
tau = 0;    % constant torque provided
t = 10;     % max time
h = 0.01;   % step size
n = t/h;    % total number of iterations

ddth = zeros(n,1);  % initialize the space to store acceleration data through n iterations
dth = zeros(n,1);   % initialize the space to store velocity data through n iterations
th = zeros(n,1);    % initialize the space to store position data through n iterations

th(1) = 0;  % initial position
dth(1) = 0;     % initial velocity
ddth(1) = 0;    % initial acceleration

kp = 4;     % define proportional control gain
kd = 4;         % define derivative control gain
ki = 1.3;         % define integrative control gain
inte = 0;       % initialize the error integral term 

% ==== calculate the acceleration given torque and defined parameter values
for i = 2:n
    dth(i) = dth(i-1)+ddth(i-1)*h; % numerical inegratin to calculate velocity from acceleration
    th(i) = th(i-1) + dth(i-1)*h;  % numerical inegratin to calculate positio from velocity
    
    e(i) = thd(i)-th(i);        % define the position error
    de(i) = dthd(i)-dth(i);     % define the velocity error
    inte  = inte + e(i)*h;      % numerical integration of the error

    tau = kp*e(i) + ki*(inte) + kd*de(i);   % torque/force control
    ddth(i) = (tau - m*g*r*cos(th(i)) - b*dth(i))/M;    % calculate the acceleration from the dynamic equation
end


plot(th, 'r.')
hold on
plot([1:n], thd([1:n]), 'b.')
xlabel('n: the number of iterations (time/h)')
ylabel('\theta: joint angle (red); \theta_d: desired trajectory (blue)')