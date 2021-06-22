function [du,u,mv] = ngmresfcn(x0,u0,du0,d,ref,T,params,nmv)
%#codegen
% This is the NMPC function by using Newton-GMRES method 
% 
% Arguments
% x0:Intial State [n,1]
% u0:Previous Input [nu*divideTime,1]
% du0:Previous rate of input [nu*divideTime,1]
% d:Disturbance [nd,1]
% ref:Refernce value of state [ny,1]
% T:Prediction time Scalar
% params:Parameter Structure
% nmv:Number of manipulated variable
% 
% Outputs
% du:Rate of input [nu*divideTime,1]
% u:Input [nu*divideTime,1]
% mv:Manipulated variable

% Initialization
du = zeros(length(du0),1,'like',du0);
u = zeros(length(u0),1,'like',u0);
mv = zeros(nmv,1);
max_iter = params.OuterIterationMax;
ts = T/params.divideNumber;                         % Sampling time during current prediction time
model = str2func(params.stateFunction);             % Function handle for the state function
dPhidx = str2func(params.jacobianTerminalFunction); % Function handle for the Jacobian of the terminal function phi
dHdx = str2func(params.jacobianHamiltonianState);   % Function handle for the Jacobian of the hamiltonian function H about state
dHdu = str2func(params.jacobianHamiltonianInput);   % Function handle for the Jacobian of the hamiltonian function H about input

% Newton-GMRES Loop
f0 = calcDiff(x0,u0,d,ref,ts,params,model,dPhidx,dHdx,dHdu);
du = fdgmres(f0,x0,u0,d,ref,du0,ts,params,model,dPhidx,dHdx,dHdu);
u = u0 + du;
for i = 1:max_iter
    f0 = calcDiff(x0,u,d,ref,ts,params,model,dPhidx,dHdx,dHdu);
    du = fdgmres(f0,x0,u,d,ref,du,ts,params,model,dPhidx,dHdx,dHdu);
    u = u + du;
end

% Extract first element from input vector
mv=u(1:nmv);

end
%EOF