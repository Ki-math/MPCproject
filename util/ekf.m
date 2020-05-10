function [xhat_new,P_new,G] = ekf(f,h,A,C,Bv,Q,R,u,y,xhat,P,Ts)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%#codegen
% Extended Kalman Filter
%
% Formulation :
% xhat(k+1) = f(x(k),u(k)) + Bv*v(k)
% y(k) = h(xhat(k),u(k)) + w(k)
% 
% Prameters
% f : Function handle for nonlinear system
% h : Function handle for nonlinear system
% A : Fucntion handle for system matrix A for state
% Bv : Function handle for system matrix Bv for system noise v
% C : Function handle for system matrix C for output
% Q : Covariance matrix Q for system noise v 
% R : Covariance matrix R for measurement noise w
% u : Input
% y : Output
% xhat : State estimation
% P : Error covarience matrix P
% Ts : Sampling time
% 
% Outputs
% xhat_new : Posteriori esitimation for state
% P_new : Posteriori error covariance matrix
% G : Kalman gain matrix
% 
% Reference : Fundamantals of Kalman Filter written by S.Adachi 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Initialization
n = length(xhat);
m = length(u);
l = length(y);
xhat_ = zeros(n,1);
xhat_new = zeros(n,1);
P_ = zeros(n);
P_new = zeros(n);
G = zeros(n,l);
Acal = A(xhat,u,Ts);
Ccal = C(xhat,u,Ts);

% Priori state estimation
xhat_ = f(xhat,u,Ts);

% Priori error covariance matrix
P_ = Acal*P*Acal'+Bv*Q*Bv';

% Kalman gain
G = (P_*Ccal')/(Ccal*P_*Ccal'+R);

% Posteriori state Esitimation 
xhat_new = xhat_+G*(y-h(xhat,u,Ts));

% Posteriori error covariance matrix
P_new = (eye(n)-G*Ccal)*P_;

end