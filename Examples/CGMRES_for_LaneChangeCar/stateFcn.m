function xdot = stateFcn(x,u,md)
% #codegen
% state_names = {'px', 'py', 'theta', 'r', 'beta'};
% normal_input_names = 'delta';
xdot = zeros(5,1);
theta = x(3);
r = x(4);
beta = x(5);
V = 15;
m = 2000;
lf = 1.4;
lr = 1.6;
I = 4000;
Kf = 12e3;
Kr = 11e3;
delta = u(1);

xdot(1) = V*cos(theta+beta);
xdot(2) = V*sin(theta+beta);
xdot(3) = r;
xdot(4) = -2*(Kf*lf^2*r+Kr*lr^2*r+Kf*V*beta*lf-Kr*V*beta*lr-Kf*V*delta*lf)/(I*V);
xdot(5) = -(2*Kf*V*beta+2*Kr*V*beta-2*Kf*V*delta+2*Kf*lf*r-2*Kr*lr*r+V^2*m*r)/(V^2*m);
end