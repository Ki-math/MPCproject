function [xhat_new,P_new,G] = kf(A,Bu,Bv,C,Q,R,u,y,xhat,P)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%#codegen
% Linear Kalman Filter
%
% Formulation :
% xhat(k+1) = A*xhat(k) + Bu*u(k) + Bv*v(k)
% y(k) = C*xhat(k) + w(k)
% 
% Arguments
% A : System matrix A for state
% Bv : System matrix Bv for noise v
% Bu : System matrix Bu for input u 
% C : System matrix C for output
% Q : Covariance matrix Q for system noise v 
% R : Covariance matrix R for measurement noise w
% u : Input
% y : Output
% xhat : State estimation
% P : Error covarience matrix P
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

% Priori state estimation
xhat_ = A*xhat+Bu*u;

% Priori error covariance matrix
P_ = A*P*A'+Bv*Q*Bv';

% Kalman gain
G = (P_*C')/(C*P_*C'+R);

% Posteriori state Esitimation 
xhat_new = xhat_+G*(y-C*xhat_);

% Posteriori error covariance matrix
P_new = (eye(n)-G*C)*P_;

end