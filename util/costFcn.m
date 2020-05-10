function J = costFcn(d,ref,params,ny,nu,nx,h)
% #codegen
% Calculating the cost function
% Arguments : 
%   d:Design variable output & input
%   ref:Reference vector
%   params:Parameter structuer
%   ny:Order of output
%   nu:Order of input
%   nx:Order of state
%   h:Function handle
% Output :
%   J:Cost function(Scalar value)

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
us = d((Hp*nx)+1:(Hp*nx)+(Hc*nu));  % Input (Dimensionless)
REFs = zeros(Hp*ny,1);
Qaug = zeros(Hp*ny);
Raug = zeros(Hc*nu);
slack = 0;
scale_mv = params.MV_scalefactor;
scale_ov = params.OV_scalefactor;
scale_state = params.State_scalefactor;

% Extract the design variables and convert to the engineering units
x = d(1:(Hp*nx)).*kron(ones(Hp,1),scale_state);                  % State
u = d((Hp*nx)+1:(Hp*nx)+(Hc*nu)).*kron(ones(Hc,1),scale_mv);     % Input
slack = d(length(d));

% Output calculation
tempy = 1:ny;
tempx = 1:nx;
tempu = 1:nu;
for i = 1:Hp
    ys(tempy) = h(x(tempx),u(tempu))./scale_ov; % Measurment function, convert to the dimensionless
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