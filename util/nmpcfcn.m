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

% Permanent variables
% persistent iA;
% persistent flag;
% if isempty(flag)
%     flag = true;
% end

% Initialization for each variable
Hp = params.PredictiveHorizon;
Hc = params.ControlHorizon;
idx_x = Hp*nx;
idx_u = Hc*nu;
idx_du = (Hc-1)*nu;
idx_y = Hp*ny;
idx_g = 2*idx_u + 2*idx_du + 2*idx_y + 1; % Last one is for the slack
idx_h = 2*idx_x;
designValue = zeros(length(d0),1,'like',d0);
mv = zeros(nu,1);
status = false;
slack = 0;
tol = 1e-6;     % Tollerance value
itermax = params.IterationMax;
mf = str2func(params.measurementFunction);  % Function handle for the state function
sf = str2func(params.stateFunction);        % Function handle for the measurement function

% Cost function
J = @(d) costFcn(d,ref,params,ny,nu,nx,mf);

% Inequality constraint
g = @(d) inqConstFcn(d,mv0,params,ny,nu,nx,mf);

% Equality constraint
h = @(d) enqConstFcn(d,x0,md,params,nu,nx,sf);

% Nonlinear constraints
nonlcon = @(d) nonlinFcn(d);
function [c,ceq] = nonlinFcn(d)
    c = g(d);
    ceq = h(d);
end

% Active set for initialization
% if isempty(iA)
%     iA = zeros((idx_g+idx_h),2);
%     inq = zeros(idx_g,1);
%     inq = g(d0);
%     for i = 1:length(iA)
%        if i <= idx_g
%            % Inequality constraints
%            if abs(inq(i)) < tol
%               iA(i,1) = 1;  % ActiveSet
%            end
%            iA(i,2) = 1; % Constraint ID for Inequality constraints
%        else
%            % Equality constraints
%            iA(i,1) = 1; % ActiveSet
%            iA(i,2) = 2; % Constraint ID for Equality constraints
%        end
%     end
% end

% Solve the SQP
if ~params.usefmincon
    % Custom sqp solver
%     if flag
%         opts = optimoptions('fmincon','Algorithm','sqp','Display','off','MaxIterations',itermax);
%         [dopt,~,exitflag] = fmincon(J,d0,[],[],[],[],[],[],nonlcon,opts);
%         if exitflag == 1
%             status = true;
%         end
%         flag = false;
%     else
%         [dopt,iA,status] = sqp(d0,J,g,h,iA,itermax);
%     end
else
    % Using fmincon with sqp
    opts = optimoptions('fmincon','Algorithm','sqp','Display','off','MaxIterations',itermax);
    [dopt,~,exitflag] = fmincon(J,d0,[],[],[],[],[],[],nonlcon,opts);
    if exitflag == 1
        status = true;
    end
end

if status
    % Feasible
    mv = dopt((idx_x+1):(idx_x+nu))./params.MV_scalefactor;
    designValue = dopt;
%     iA(1:idx_g,1) = 0;
    slack = dopt(length(dopt));
else
    % Infeasible
    mv = mv0;
    designValue = d0;
end

end
%EOF