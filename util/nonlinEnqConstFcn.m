function h = nonlinEnqConstFcn(d,x0,md,params,nu,nx,f)
%#codegen
% Calculating the Equality constraint
% Arguments:
%   d:Design variable
%   x0:Initial state
%   md:Measurement disturbance
%   params:Parameter structuer
%   nu:Order for input
%   nx:Order for state
% Outputs:
%   h:Equality constraint vector

% Initialization 
Hp = params.PredictiveHorizon;
Hc = params.ControlHorizon;
scale_mv = params.MV_scalefactor;
scale_state = params.State_scalefactor;
x = zeros(Hp*nx,1);
u = zeros(Hc*nu,1);
h = zeros(Hp*nx,1);

% Extract the design variables and convert to the engineering units
x = d(1:(Hp*nx)).*kron(ones(Hp,1),scale_state);                 % State
u = d((Hp*nx)+1:(Hp*nx)+(Hc*nu)).*kron(ones(Hc,1),scale_mv);    % Input

% Construct the equality constraint about state transition, they are converted to dimensionless
idx_x = 1:nx;
idx_u = 1:nu;
h(idx_x) = (x(idx_x) - f(x0,u(idx_u),md))./scale_state;   %x_1 - f(x_0,u_0)
if Hc > 1
    idx_u = idx_u + nu;
end
if Hp > 1
    idx_x = idx_x + nx;
end

for i = 2:Hp
    h(idx_x) = (x(idx_x) - f(x(idx_x-nx),u(idx_u),md))./scale_state; %x_k - f(x_k-1,u_k-1)
    if i < Hc
        idx_u = idx_u + nu;
    end
    if Hp > 1
        idx_x = idx_x + nx;
    end
end

end
%EOF