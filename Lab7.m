clc; clear;
% ESET 462 - Lab 7
% Keith Fritz

% ~~~~~Section 1: Bode Diagram~~~~~
% Example
H = tf([1 0.1 7.5],[1 0.12 9 0 0]);
figure(1)
bode(H)
% System I)
S1 = tf([25],[1 4 25]);
figure(2)
bode(S1)
% System II)
S2 = tf([9 1.8 9],[1 1.2 9 0]);
figure(3)
bode(S2)
% System III)
A = [0 -1; -25 -4];
B = [1 1; 0 1];
C = [1 0; 0 1];
D = [0];
figure(4)
bode(A,B,C,D)


% ~~~~~Section 2: PID Control Loop with Dead Time~~~~~
s = tf('s');
% Example
P = exp(-2.6*s)*(s+3) / (s^2+0.3*s+1);
C = 0.06 * (1 + 1/s);
T = feedback(P*C,1)
figure(5)
step(T)
% System I)
P1 = exp(0*s)*(s+3) / (s^2+0.3*s+1);
C1 = 0.06 * (1 + 1/s);
T1 = feedback(P1*C1,1)
figure(6)
step(T1)
% System II)
P2 = exp(-5*s)*(s+3) / (s^2+0.3*s+1);
C2 = 0.06 * (1 + 1/s);
T2 = feedback(P2*C2,1)
figure(7)
step(T2)
% System III)
P3 = exp(-9*s)*(s+3) / (s^2+0.3*s+1);
C3 = 0.06 * (1 + 1/s);
T3 = feedback(P3*C3,1)
figure(8)
step(T3)
% System IV)
P4 = exp(-10*s)*(s+3) / (s^2+0.3*s+1);
C4 = 0.06 * (1 + 1/s);
T4 = feedback(P4*C4,1)
figure(9)
step(T4)