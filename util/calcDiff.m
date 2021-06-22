function F = calcDiff(x,u,d,ref,ts,params,model,dPhidx,dHdx,dHdu)
%#codegen
% Calculation of the nonlinear algebraic equation by solve the Euler
% Lagrange equation
% 
% Arguments
% x:State
% u:Input
% d:Disturbance
% ref:Refernce value of state
% ts:Sampling time
% params:Parameter structuer
% model:Function handle of the state transition
% dPhidx:Function handle of the Jacobian of the terminal function phi
% dHdx:Function handle for the Jacobian of the hamiltonian function H about state
% dHdu:Function handle for the Jacobian of the hamiltonian function H about input
% 
% Outputs
% F:Nonlinear algebraic equation vector

% Common parameters
nx = length(x);
nu = length(params.initialConditionInputs);
dv = params.divideNumber;

% Forward simulation for the state variables by using Runge-kutta
X = zeros(nx*dv,1);
idx_x = 1:nx;
idx_u = 1:nu;
X(idx_x) = x;
idx_x = idx_x + nx;
for j = 1:dv-1
    X(idx_x) = RK4(model,X(idx_x-nx),u(idx_u),d,ts);
    if j < dv-1
        idx_x = idx_x + nx;
        idx_u = idx_u + nu;
    end
end

% Backward simulation for the lagurange variables by using Forward difference approximation
lamda = zeros(nx*dv,1);
lamda(idx_x) = dPhidx(X(idx_x),ref,params);    % Value at the terminal point
for k = dv:-1:2
   lamda(idx_x-nx) = lamda(idx_x) + dHdx(X(idx_x-nx),lamda(idx_x),u(idx_u),d,ref,params)*ts;
   idx_x = idx_x - nx;
   if k > 2
        idx_u = idx_u - nu;
   end
end

% Caluculation for Non-linear algebraic eqns
F = zeros(nu*dv,1);
for i = 1:dv
   F(idx_u) = dHdu(X(idx_x),lamda(idx_x),u(idx_u),d,ref,params);
   idx_x = idx_x + nx;
   idx_u = idx_u + nu;
end

end
