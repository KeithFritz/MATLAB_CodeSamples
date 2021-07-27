% MXET 400
% HW 6
% Keith Fritz
clc; clear;

% USE 'x3.m' AND 'Jac3.m' AND INPUT Xend ANALYTICAL EXPRESSION FOR USE ON
% FINAL

% Desired end-effector position
xd = [2.1213, 2.1213, 1]';

% Initialization
th = [0.5, 0.5, 1]';
ed = 0.01;

% Define error vector
e = xd-x3(th);

% Calculate the error as the vector norm of 3
error = norm(e);

% Update 'th' until the error condition is satisfied
while error > ed
    th = th + pinv(Jac3(th))*e;
    e = xd-x3(th);
    error = norm(e);
end

% Display the final result in degrees
Result = [th(1)*180/pi, th(2)*180/pi, th(3)]'

% Verify the solution by
x3(th)