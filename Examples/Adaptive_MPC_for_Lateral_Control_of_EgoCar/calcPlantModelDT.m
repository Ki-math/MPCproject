function [A,B,Bd,C,U,MD,Y,X,f0] = calcPlantModelDT(Ts,Vx,u,x)
% Model parameters
m = 1575;
Iz = 2875;
lf = 1.2;
lr = 1.6;
Cf = 19000;
Cr = 33000;

% Continuous-time model
Ac = [-(2*Cf+2*Cr)/m/Vx, 0, -Vx-(2*Cf*lf-2*Cr*lr)/m/Vx, 0;
     0, 0, 1, 0;
     -(2*Cf*lf-2*Cr*lr)/Iz/Vx, 0, -(2*Cf*lf^2+2*Cr*lr^2)/Iz/Vx, 0;
     1, Vx, 0, 0];
Bc = [2*Cf/m 0 2*Cf*lf/Iz 0]';   
Cc = [0 0 0 1; 0 1 0 0];
Dc = zeros(2,1);

% Generate discrete-time model
nx = size(Ac,1);
nu = size(Bc,2);
A = eye(nx)+Ts*Ac;  %Euler approximation
B = Ts*Bc;
Bd = zeros(nx,1);
C = Cc;
D = Dc;

% Nominal conditions for discrete-time plant
X = x;
U = u;
MD = 0;
Y = C*X+D*U;
f0 = Ts*(Ac*X+Bc*U);
% X = zeros(nx,1);
% U = zeros(nu,1);
% Y = zeros(size(C,1),1);
% f0 = zeros(nx,1);%Ts*(A*X+B*U);

