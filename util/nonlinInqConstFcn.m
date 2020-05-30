function g = nonlinInqConstFcn(d,x0,md,params,ny,nu,nx,f,h)
%#codegen
% Calculating the nonlinear inequality constraint
% Arguments:
%   d:Design variable
%   x0:Initial state
%   md:Measurement disturbance
%   params:Parameter structuer
%   ny:Order for output
%   nu:Order for input
%   nx:Order for state
%   f:Function handle of state function
%   h:Function handle of measurement function
% Outputs:
%   g:Inequality Constraint vector

Hp = params.PredictiveHorizon;
Hc = params.ControlHorizon;
ymin = params.OV_min;
ymax = params.OV_max;

% Initialization 
x = zeros(Hp*nx,1);
u = zeros(Hc*nu,1);
ys = zeros(Hp*ny,1);
Vmax = zeros(Hp*ny,1);
Vmin = zeros(Hp*ny,1);
Vmax = params.softConstRelaxation(1)*ones(Hp*ny,1);
Vmin = params.softConstRelaxation(2)*ones(Hp*ny,1);
idx_y = 2*Hp*ny;
g = zeros(idx_y,1);
scale_mv = params.MV_scalefactor;
scale_ov = params.OV_scalefactor;
slack = 0;

% Extract the design variables and convert to the engineering units
u = d(1:Hc*nu).*kron(ones(Hc,1),scale_mv);                      % Input
slack = d(length(d));                                           % Slack

% Calculation for the Output
tempy = 1:ny;
tempx = 1:nx;
tempu = 1:nu;
for i = 1:Hp
    x(tempx,1) = f(x0,u(tempu,1),md);                       % State transition
    ys(tempy,1) = h(x(tempx,1),u(tempu,1))./scale_ov;       % Measurment function, convert to the dimensionless
    tempy = tempy + ny;
    tempx = tempx + nx;
    if i < Hc
        tempu = tempu + nu;
    end
end

% Definition of Nonlinear inequality constraint g, dimension less
g = [ys - kron(ones(Hp,1),ymax./scale_ov) - slack*Vmax;
     -ys + kron(ones(Hp,1),ymin./scale_ov) - slack*Vmin
     ];

end
%EOF