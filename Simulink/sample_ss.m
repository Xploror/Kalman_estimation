 % Inverted pendulum model (linear)
clc;clear;

% State variables(x) [phi;theta;psi;phi_dot;theta_dot;psi_dot] 3DoF
% Input matrix(u) [F1;F2;F3;F4] forces of each motor
% Output matrix(y) same as state variables

d = 0.243*cos(45*pi/180);   % in m
c = 0.243;    % in m
Ix = 0.0213;
Iy = 0.02217;  % All 3 MOI in kgm^2
Iz = 0.0282;

A_m = [0 0 0 1 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0];
B_m = [0 0 0 0; 0 0 0 0; 0 0 0 0; 0 d/Ix 0 -d/Ix; d/Iy 0 -d/Iy 0; -c/Iz c/Iz -c/Iz c/Iz];
C_m = eye(6);
D_m = zeros(size(C_m,1),size(B_m,2));

op_ss = ss(A_m, B_m, C_m, D_m);

% Discrete Time Model
dT = 0.05;

sys_d = c2d(op_ss,dT,'zoh');

A_d = sys_d.A;
B_d = sys_d.B;
C_d = sys_d.C;
D_d = sys_d.D;

G = eye(6);
H = zeros(size(C_m,1),size(C_m,1));

Qcov = diag(0.15*ones(1,size(A_m,1)));
Rcov = diag(0.05*ones(1,size(A_m,1)));

sys_kf = ss(A_d, [B_d G], C_d, [D_d H], dT);  % in form x_dot = A_d*x + B_d*u + G*w 
                            %  y = C_d*x + D_d*u + H*w + v

[kest,L,P] = kalman(sys_kf,Qcov,Rcov,0);

L_bar = (A_d*P*C_d)/(C_d*P*C_d' + Rcov);  % Ricatti equation

err = norm(abs(L_bar-L));

% Stability of filter (i.e eigenvalue of matrix corresponding to prior estimate of state)
poles = eig(A_d - L*C_d);