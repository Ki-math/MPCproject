function [xopt,lamda,status] = interiorPointMethod_inq(x,lamda,s,Q,c,A,b,iterMax)
% codegen
%Interior Point Method via Predictor-Corrector Method with Inequality Constraint
%Arguments
%x:Initial guess of design variable
%lamda:Lagrange variable of Inequality constraint
%s:Slack varibale of Inequality constraint
%Q:Weight of quadratic term (hessian)
%c:Coefficient of 1st order term (jacobian)
%A:Inequality coefficient of A*x>=b
%b:Inequality constant of A*x>=b
%iterMax:Max iteration number
%
%Output
%xopt:Resolution
%lamda:Lagrange variable of inequality constaints
%status: Ture:Optimized solution False:semi-optimized solution
%
%Cost function form
% J = 0.5*x'*Q*x + c'*x
% A*x >= b

% Order of each variable
% Design variable
n = length(x);

% Inequality Constraint
m = size(A,1);

% Initialization for each variable
% Design variable
xopt = zeros(n,1);

% e is ones vector
e = ones(m,1);

% Residue vector
rd = zeros(n,1);
rinq = zeros(m,1);
rsl = zeros(m,1);

% Search direction
dx_aff = zeros(n,1);
dx = zeros(n,1);
dlamda_aff = zeros(m,1);
dlamda = zeros(m,1);
ds_aff = zeros(m,1);
ds = zeros(m,1);

% Variable fpr Cholesky decomposition and augemented system
Q_bar = zeros(n);
r_bar = zeros(n,1);
g_bar = zeros(n,1);
L = zeros(n);

% Convergence flag for Optimization
status = false;

% Complementarity measure
mu = 0;
mu_aff = 0;

% Centaring parameter
sigma = 0;

% Scaling parameter
eta = 0.95;

% Small tollerance
tol = 1e-6;

% Flag for the Cholesky decomposition
flag = false;

% Optimization Loop
for j = 1:iterMax

% Residue vector
rd = Q*x+c-A'*lamda;
rinq = s-A*x+b;
rsl = s.*lamda;

% Complementarity measure
mu = (s'*lamda)/m;

% Stop Condition
if norm(rd)<=tol && norm(rinq)<=tol && norm(rsl)<=tol && abs(mu)<=tol 
   status = true;
   break;
end

% Preditor-Corrector Step
% Inequality constraint
% Solve the system
Q_bar = Q + A'*(diag(lamda./s))*A;
r_bar = A'*((rsl-lamda.*rinq)./s);
g_bar = -(r_bar+rd);

% Check whethrt or not the Q_bar matris is positive define.
% If the Q_bar is negative define, the Psuedo inverse 'pinv' is used instead of the Cholesky decomposition 'chol'.
chk = eig(Q_bar);
for k = 1:length(chk)
    if chk(k) < 0 || chk(k) == inf
        flag = true;
    end
end
if flag
    % Solve by Psuedo inverse
    dx_aff = pinv(Q_bar)*g_bar;
else
    % Solve by Cholesky decomposition
    L = chol(Q_bar,'lower');
    dx_aff = L'\(L\g_bar); 
end

ds_aff = -rinq+A*dx_aff;
dlamda_aff = -(rsl+lamda.*ds_aff)./s;
   
% Calc the centering parameter sigma
a_aff = 1;
for i = 1:length(dlamda_aff)
    if dlamda_aff(i) < 0
        a_aff = min(a_aff,-lamda(i)./dlamda_aff(i));
    end
end
for i = 1:length(ds_aff)
    if ds_aff(i) < 0
        a_aff = min(a_aff,-s(i)./ds_aff(i));
    end
end
   
% Complementarity measure of affine acaling
mu_aff = (s+a_aff*ds_aff)'*(lamda+a_aff*dlamda_aff)/m;
   
% Centaring parameter
sigma = (mu_aff/mu)^3;
   
% Corrector Step
rsl = rsl + ds_aff.*dlamda_aff - sigma*mu*e;
   
% Solve the serch direction
r_bar = A'*((rsl-lamda.*rinq)./s);
g_bar = -(r_bar+rd);
dx = L'\(L\g_bar);
ds = -rinq+A*dx;
dlamda = -(rsl+lamda.*ds)./s;
   
% Calc the step length
a = 1;
for i = 1:length(dlamda)
    if dlamda(i) < 0
        a = min(a,-lamda(i)./dlamda(i));
    end
end
for i = 1:length(ds)
    if ds(i) < 0
        a = min(a,-s(i)./ds(i));
    end
end
      
% Update Solution
x = x + eta*a*dx;
lamda = lamda + eta*a*dlamda;
s = s + eta*a*ds;

end

if ~status
    %Cannot find best solutions
%     disp('Cannot find optimized solution!')
end

% Apply the solution
xopt = x;

end