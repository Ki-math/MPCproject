function g = inqConstFcn(d,mv0,params,ny,nu,nx,h)
%#codegen
% Calculating the inequality constraint
% Arguments:
%   d:Design variable
%   mv0:Last Input
%   params:Parameter structuer
%   ny:Order for output
%   nu:Order for input
%   nx:Order for state
%   h:Function handle
% Outputs:
%   g:Inequality Constraint vector

Hp = params.PredictiveHorizon;
Hc = params.ControlHorizon;
umin = params.MV_min;
umax = params.MV_max;
dumin = params.dMV_min;
dumax = params.dMV_max;
ymin = params.OV_min;
ymax = params.OV_max;

% Initialization 
x = zeros(Hp*nx,1);
u = zeros(Hc*nu,1);
du = zeros(Hc*nu,1);
ys = zeros(Hp*ny,1);
us = zeros(length(u),1,'like',u);
us = d((Hp*nx)+1:(Hp*nx)+(Hc*nu));  % Input (Dimensionless)
dus = zeros(length(du),1,'like',du);
slack = 0;
Vmax = zeros(Hp*ny,1);
Vmin = zeros(Hp*ny,1);
Vmax = params.softConstRelaxation(1)*ones(Hp*ny,1);
Vmin = params.softConstRelaxation(2)*ones(Hp*ny,1);
idx_u = Hc*nu;
idx_du = (Hc-1)*nu;
idx_y = Hp*ny;
idx_g = 2*idx_u + 2*idx_du + 2*idx_y + 1;
g = zeros(idx_g,1);
scale_mv = params.MV_scalefactor;
scale_ov = params.OV_scalefactor;
scale_state = params.State_scalefactor;

% Extract the design variables and convert to the engineering units
x = d(1:(Hp*nx)).*kron(ones(Hp,1),scale_state);                 % State
u = d((Hp*nx)+1:(Hp*nx)+(Hc*nu)).*kron(ones(Hc,1),scale_mv);    % Input
slack = d(length(d));

% Calculation for the Input rate, Engneering units
du(1:nu) = u(1:nu) - mv0;   % Difference between the previous input and the first element of input sequence for current step
du((nu+1):(Hc*nu)) = u((nu+1):(Hc*nu)) - u(1:(Hc*nu)-nu);
dus = du./kron(ones(Hc,1),scale_mv);    % Convert to the dimensionless

% Calculation for the Output
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

% Definition of Nonlinear inequality constraint g, dimension less
g = [us - kron(ones(Hc,1),umax./scale_mv);
     -us + kron(ones(Hc,1),umin./scale_mv);
     dus - kron(ones(Hc,1),dumax./scale_mv);
     -dus + kron(ones(Hc,1),dumin./scale_mv);
     ys - kron(ones(Hp,1),ymax./scale_ov) - slack*Vmax;
     -ys + kron(ones(Hp,1),ymin./scale_ov) - slack*Vmin;
     -slack
     ];

end
%EOF