% MXET 400
% HW 8
% Keith Fritz
clc; clear;

% Define parameters
m = 2;
l = 8;
w = 0.5;
h = 0.5;
g = 9.8;
density = 1;

% Define spatial intertia matrices (same for all links)
Ixx = (m/12)*(w^2 + h^2);
Iyy = (m/12)*(l^2 + h^2);
Izz = (m/12)*(l^2 + w^2);
I = [Ixx 0 0; 0 Iyy 0; 0 0 Izz];
G = [I, zeros(3,3); zeros(3,3), m*eye(3)];

% Define screw axis
S = [0 0 1 0 0 0]';

% Define zero configurations
M1 = [1 0 0 l*1; 0 1 0 0; 0 0 1 0; 0 0 0 1];
M2 = [1 0 0 l*2; 0 1 0 0; 0 0 1 0; 0 0 0 1];
M3 = [1 0 0 l*3; 0 1 0 0; 0 0 1 0; 0 0 0 1];
M4 = [1 0 0 l*4; 0 1 0 0; 0 0 1 0; 0 0 0 1];
M5 = [1 0 0 l*5; 0 1 0 0; 0 0 1 0; 0 0 0 1];
M6 = [1 0 0 l*6; 0 1 0 0; 0 0 1 0; 0 0 0 1];
M7 = [1 0 0 l*7; 0 1 0 0; 0 0 1 0; 0 0 0 1];
M8 = [1 0 0 l*8; 0 1 0 0; 0 0 1 0; 0 0 0 1];
M9 = [1 0 0 l*9; 0 1 0 0; 0 0 1 0; 0 0 0 1];
M10 = [1 0 0 l*10; 0 1 0 0; 0 0 1 0; 0 0 0 1];
% Calculate the screw axes defined at each link frame
A1 = Ad(inv(M1)) * S;
A2 = Ad(inv(M2)) * S;
A3 = Ad(inv(M3)) * S;
A4 = Ad(inv(M4)) * S;
A5 = Ad(inv(M5)) * S;
A6 = Ad(inv(M6)) * S;
A7 = Ad(inv(M7)) * S;
A8 = Ad(inv(M8)) * S;
A9 = Ad(inv(M9)) * S;
A10 = Ad(inv(M10)) * S;


% Define given robot states and initial conditions
th = pi/6;          % positions
dth = 0.1;          % velocities
ddth = 0.01;        % accelerations

T11 = eye(4);
F11 = zeros(6,1);
V0 = [0 0 0 0 0 0]';
dV0 = [0 -g 0 0 0 0]';

% FORWARD ITERATIONS

% i = 1
T1 = expm(-mSkew(A1)*th)*M1;
V1 = Ad(T1)*V0 + dth*A1;
dV1 = Ad(T1)*dV0 + dth*sAd(V1)*A1+ddth*A1;
% i = 2
T2 = expm(-mSkew(A2)*th)*M2;
V2 = Ad(T2)*V1 + dth*A2;
dV2 = Ad(T2)*dV1 + dth*sAd(V2)*A2+ddth*A2;
% i = 3
T3 = expm(-mSkew(A3)*th)*M3;
V3 = Ad(T3)*V2 + dth*A3;
dV3 = Ad(T3)*dV2 + dth*sAd(V3)*A3+ddth*A3;
% i = 4
T4 = expm(-mSkew(A4)*th)*M4;
V4 = Ad(T4)*V3 + dth*A4;
dV4 = Ad(T4)*dV3 + dth*sAd(V4)*A4+ddth*A4;
% i = 5
T5 = expm(-mSkew(A5)*th)*M5;
V5 = Ad(T5)*V4 + dth*A5;
dV5 = Ad(T5)*dV4 + dth*sAd(V5)*A5+ddth*A5;
% i = 6
T6 = expm(-mSkew(A6)*th)*M6;
V6 = Ad(T6)*V5 + dth*A6;
dV6 = Ad(T6)*dV5 + dth*sAd(V6)*A6+ddth*A6;
% i = 7
T7 = expm(-mSkew(A7)*th)*M7;
V7 = Ad(T7)*V6 + dth*A7;
dV7 = Ad(T7)*dV6 + dth*sAd(V7)*A7+ddth*A7;
% i = 8
T8 = expm(-mSkew(A8)*th)*M8;
V8 = Ad(T8)*V7 + dth*A8;
dV8 = Ad(T8)*dV7 + dth*sAd(V8)*A8+ddth*A8;
% i = 9
T9 = expm(-mSkew(A9)*th)*M9;
V9 = Ad(T9)*V8 + dth*A9;
dV9 = Ad(T9)*dV8 + dth*sAd(V9)*A9+ddth*A9;
% i = 10
T10 = expm(-mSkew(A10)*th)*M10;
V10 = Ad(T10)*V9 + dth*A10;
dV10 = Ad(T10)*dV9 + dth*sAd(V10)*A10+ddth*A10;


% BACKWARD ITERATIONS

%i = 10
F10 = Ad(T11)'*F11 + G*dV10 - sAd(V10)'*(G*V10);
tau10 = F10'*A10
%i = 9
F9 = Ad(T10)'*F10 + G*dV9 - sAd(V9)'*(G*V9);
tau9 = F9'*A9
%i = 8
F8 = Ad(T9)'*F9 + G*dV8 - sAd(V8)'*(G*V8);
tau8 = F8'*A8
%i = 7
F7 = Ad(T8)'*F8 + G*dV7 - sAd(V7)'*(G*V7);
tau7 = F7'*A7
%i = 6
F6 = Ad(T7)'*F7 + G*dV6 - sAd(V6)'*(G*V6);
tau6 = F6'*A6
%i = 5
F5 = Ad(T6)'*F6 + G*dV5 - sAd(V5)'*(G*V5);
tau5 = F5'*A5
%i = 4
F4 = Ad(T5)'*F5 + G*dV4 - sAd(V4)'*(G*V4);
tau4 = F4'*A4
%i = 3
F3 = Ad(T4)'*F4 + G*dV3 - sAd(V3)'*(G*V3);
tau3 = F3'*A3
%i = 2
F2 = Ad(T3)'*F3 + G*dV2 - sAd(V2)'*(G*V2);
tau2 = F2'*A2
%i = 2
F1 = Ad(T2)'*F2 + G*dV1 - sAd(V1)'*(G*V1);
tau1 = F1'*A1
