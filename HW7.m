% MXET 400
% HW 7
% Keith Fritz

% NEWTON_EULER DYNAMICS

% Define parameters
m1 = 2;
m2 = 2;
L1 = 4;
L2 = 4;
g = 9.8;

% Define spatial intertia matrices
I1 = [1 0 0; 0 2 0; 0 0 2];
G1 = [I1, zeros(3,3); zeros(3,3), m1*eye(3)];
I2 = [1 0 0; 0 2 0; 0 0 2];
G2 = [I2, zeros(3,3); zeros(3,3), m2*eye(3)];

% Define screw axes
S1 = [0 0 1 0 0 0]';
S2 = [0 0 1 0 0 0]';

% Define zero configurations
M02 = [1 0 0 L1; 0 1 0 0; 0 0 1 0; 0 0 0 1];
M01 = [1 0 0 L1+L2; 0 1 0 0; 0 0 1 0; 0 0 0 1];
M10 = inv(M01);
M12 = M10*M02;
M21 = inv(M12);

% Calculate the screw axes defined at each link frame
A1 = Ad(inv(M01)) * S1;
A2 = Ad(inv(M02)) * S2;

% Define given robot states and initial conditions
th = [pi/4; pi/3];  % positions
dth = [1; 1];       % velocities
ddth = [0.1; 0.1];      % accelerations

T23 = eye(4);
T32 = eye(4);
F3 = zeros(6,1);
V0 = [0 0 0 1 0 0]';
dV0 = [0 0 0 0 0 -g]';

% FORWARD ITERATIONS

% i = 1
T10 = expm(-mSkew(A1)*th(1))*M10;
V1 = Ad(T10)*V0 + dth(1)*A1;
dV1 = Ad(T10)*dV0 + dth(1)*sAd(V1)*A1+ddth(1)*A1;

% i = 2
T21 = expm(-mSkew(A2)*th(2))*M21;
V2 = Ad(T21)*V1 + dth(2)*A2;
dV2 = Ad(T21)*dV1 + dth(2)*sAd(V2)*A2+ddth(2)*A2;

% BACKWARD ITERATIONS

%i = 2
F2 = Ad(T32)'*F3 + G2*dV2 - sAd(V2)'*(G2*V2)
tau2 = F2'*A2

% i = 1
F1 = Ad(T21)'*F2 + G1*dV1 - sAd(V1)'*(G1*V1)
tau1 = F1'*A1
