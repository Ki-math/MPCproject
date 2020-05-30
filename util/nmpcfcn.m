function [designValue,mv,slack,status] = nmpcfcn(d0,x0,mv0,md,ref,params,ny,nu,nx)
%#codegen
% Nonlinear MPC funtion : Author by Kohei Iwamoto
% Created in 7/5/2020
%
% Arguments
% d0:Initial design variable (which icludes the state and the manipulated variable), Engneering unit
% x0:Initial observed value(OV), Engineering unit
% mv0:Last manipulated value(MV), Engineering unit
% md:Measurement Disturbanceinput(DV) This is the constant value for entire future steps, Engineering unit
% ref:Reference value, Enginerring unit
% params:Parameter structuer
% ny:Order of OV
% nu:Order of MV
% nx:Order of state
% 
% Outputs
% desginValue:Design variable vector, Engineering unit
% mv:Optimized MV, Engineering unit
% slack:Slack variable for the soft constrainsts
% status:Optimization flag (1:feasible 0:infeasible)

% Initialization for each variable
designValue = zeros(length(d0),1,'like',d0);
mv = zeros(nu,1);
status = false;
slack = 0;
itermax = params.IterationMax;
mf = str2func(params.measurementFunction);  % Function handle for the measurement function
sf = str2func(params.stateFunction);        % Function handle for the state function

% Cost function
J = @(d) costFcn(d,x0,ref,md,params,ny,nu,nx,sf,mf);
% cost = @(d) calcCostFcn(d,x0,ref,md,params,ny,nu,nx,sf,mfJ);
% function [Jc,dJc] = calcCostFcn(d,x0,ref,md,params,ny,nu,nx,sf,mf,J)
%    Jc = J(d);
%    dJc = calcJacobian(d,J);
% end

% Linear Inequality constraint
[A,b] = linInqConstFcn(d0,mv0,params,nu);

% Nonlinear Inequality constraint
g = @(d) nonlinInqConstFcn(d,x0,md,params,ny,nu,nx,sf,mf);

% Nonlinear Equality constraint
% h = @(d) nonlinEnqConstFcn(d,x0,md,params,nu,nx,sf);

% Nonlinear constraints
nonlcon = @(d) nonlinFcn(d);
function [c,ceq] = nonlinFcn(d)
    c = g(d);
    ceq = [];
end

% Bound constraint
lb = -inf(length(d0),1); lb(length(lb),1) = 0;
ub = inf(length(d0),1);

% Solve NP by using the fmincon with sqp
% opts = optimoptions('fmincon','Algorithm','sqp','Display','off','SpecifyObjectiveGradient',true,'MaxIterations',itermax);
opts = optimoptions('fmincon','Algorithm','sqp','Display','off','MaxIterations',itermax);
[dopt,~,exitflag] = fmincon(J,d0,A,b,[],[],lb,ub,nonlcon,opts);
if exitflag > 0
    status = true;
end

if status
    % Feasible
    mv = dopt(1:nu).*params.MV_scalefactor;
    designValue = dopt;
    slack = dopt(length(dopt));
else
    % Infeasible
    mv = mv0;
    designValue = d0;
end

end
%EOF