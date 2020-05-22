function J = costFcn(d,x0,ref,md,params,ny,nu,nx,f,h)
% #codegen
% Calculating the cost function
% Arguments : 
%   d:Design variable (input)
%   x0:Initial state
%   ref:Reference vector
%   md:Measurement Disturbance
%   params:Parameter structuer
%   ny:Order of output
%   nu:Order of input
%   nx:Order of state
%   f:Function handle of state transition
%   h:Function handle of measurement function
% Output :
%   J:Cost function(Scalar value)
%   dJ:Gradient of cost fiunction

Hp = params.PredictiveHorizon;
Hc = params.ControlHorizon;
Q = params.Q;
R = params.R;
S = params.rho; % Weight for the slack

% Initialization 
J = 0;
x = zeros(Hp*nx,1);
u = zeros(Hc*nu,1);
ys = zeros(Hp*ny,1);
us = zeros(length(u),1,'like',u);
us = d(1:Hc*nu,1);                  % Input (Dimensionless)
REFs = zeros(Hp*ny,1);
Qaug = zeros(Hp*ny);
Raug = zeros(Hc*nu);
slack = 0;
scale_mv = params.MV_scalefactor;
scale_ov = params.OV_scalefactor;

% Extract the design variables and convert to the engineering units
u = us.*kron(ones(Hc,1),scale_mv);
slack = d(length(d));

% Output calculation
tempy = 1:ny;
tempx = 1:nx;
tempu = 1:nu;
for i = 1:Hp
    x(tempx,1) = f(x0,u(tempu,1),md);                 % State transition
    ys(tempy,1) = h(x(tempx,1),u(tempu,1))./scale_ov; % Measurment function, convert to the dimensionless
    x0 = x(tempx,1);                                  % Update initial state
    tempy = tempy + ny;
    tempx = tempx + nx;
    if i < Hc
        tempu = tempu + nu;
    end
end

% Calculating the cost function
REFs = kron(ones(Hp,1),ref./scale_ov);  % Convert to the dimensionless
Qaug = kron(eye(Hp),Q);
Raug = kron(eye(Hc),R);
J = 0.5 * ((ys-REFs)'*Qaug*(ys-REFs) + us'*Raug*us + slack^2*S);
end