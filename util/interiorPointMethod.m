function [xopt,lamda,rho,status] = interiorPointMethod(x,Q,c,A,b,Aeq,beq)
% codegen
%Interior Point Method via Predictor-Corrector Method
%
%Arguments
%x:Initial guess of design variable
%Q:Weight of quadratic term (hessian)
%c:Coefficient of 1st order term (jacobian)
%A:Inequality coefficient of A*x>=b
%b:Inequality constant of A*x>=b
%Aeq:Equality coefficient of Aeq*x=b
%beq:Equality Constant of Aeq*x=b
%
%Output
%xopt:Resolution
%lamda:Lagrange variable of inequality constaints
%mu:Lagrange variable of equality constaints
%status: Ture:Optimized solution False:semi-optimized solution
%
%Cost function form
% J = 0.5*x'*Q*x + c'*x
% Aeq*x = beq
% A*x > b

% Order of each variable
% Design variable
n = length(x);   

% Inequality constraint
if isempty(A)
    m = 0;
    A_in = zeros(1,n);
    b_in = 0;
else
    m = size(A,1);
    A_in = A;
    b_in = b;
end

% Equality constraint
if isempty(Aeq)
    l = 0;
    Aeq_in = zeros(1,n);
    beq_in = 0;
else
    l = size(Aeq,1);
    Aeq_in = Aeq;
    beq_in = beq;
end

% Initialization
% Design variable
xopt = zeros(n,1);

% lamda : Lagrage variable for inequality constraint
% s : Slack variable for converting inequality constraint to equality constraint
if m == 0
    lamda = 0;
    s = 0;
else
    % Initialize as 1 for each variable
    lamda = ones(m,1); 
    s = ones(m,1);
end

% e is ones vector
e = ones(m,1);

% Lagrange variable for equality constraint
if l == 0
    rho = 0;
else
    rho = zeros(l,1);
end

% Convergence flag for Optimization
status = false;

% Complementarity measure
mu = 0;
mu_aff = 0;

% Centaring parameter
sigma = 0;

% Scaling parameter
eta = 0.95;

%Small tollerance
tol = 1e-6;

%Max Iterlation
iterMax = 200;

% Residue vector
rd = zeros(n,1);
if m == 0
    rinq = 0;
    rsl = 0;
else
    rinq = zeros(m,1);
    rsl = zeros(m,1);
end
if l == 0
    req = 0;
else
    req = zeros(l,1);
end

% Search direction
dx_aff = zeros(n,1);
dx = zeros(n,1);
if l == 0
    drho = 0;
else
    drho = zeors(l,1);
end
if m == 0
    dlamda_aff = 0;
    dlamda = 0;
    ds_aff = 0;
    ds = 0;
else
    dlamda_aff = zeros(m,1);
    dlamda = zeros(m,1);
    ds_aff = zeros(m,1);
    ds = zeros(m,1);
end

if l ~= 0 && m == 0
    J = zeros(n+l);
    r = zeros(n+l,1);
    d = zeros(n+l,1);
elseif l ~= 0 && m ~= 0
    J = zeros(n+l+m);
    r = zeros(n+l+m,1);
    d = zeros(n+l+m,1);
else
    J = 0;
    r = 0;
    d = 0;
end

if m ~= 0 && l == 0
    Q_bar = zeros(n);
    r_bar = zeros(n,1);
    g_bar = zeros(n,1);
    L = zeros(n);
else
    Q_bar = 0;
    r_bar = 0;
    g_bar = 0;
    L = 0;
end
    

% Optimization Loop
for j = 1:iterMax

% Residue vector
if m == 0 && l == 0
   % No constraint
   rd = Q*x+c;
   % Stop condition
   if norm(rd)<=tol
       status = true;
       break;
   end
   
elseif m == 0 && l ~= 0
   % Equality constraint
   rd = Q*x+c-Aeq_in'*rho;
   req = Aeq_in*x-beq_in;
   % Stop condition
   if norm(rd)<=tol && norm(req)<=tol
       status = true;
       break;
   end
    
elseif m ~= 0 && l == 0
   % Inequality constraint
   rd = Q*x+c-A_in'*lamda;
   rinq = s-A_in*x+b_in;
   rsl = s.*lamda;
   mu = (s'*lamda)/m;  % Complementarity measure
   % Stop Condition
   if norm(rd)<=tol && norm(rinq)<=tol && norm(rsl)<=tol && abs(mu)<=tol 
      status = true;
      break;
   end
   
else
   % Full formulation
   rd = Q*x+c-A_in'*lamda-Aeq_in'*rho;
   req = -Aeq_in*x+beq_in;
   rinq = s-A_in*x+b_in;
   rsl = s.*lamda;
   mu = (s'*lamda)/m;  % Complementarity measure
   % Stop Condition
   if norm(rd)<=tol && norm(req)<= tol && norm(rinq)<=tol && norm(rsl)<=tol && abs(mu)<=tol 
      status = true;
      break;
   end 
end

% Preditor-Corrector Step
if m == 0 && l == 0
   % No constarint
   % Solve the Matrix
   dx = Q\(-rd);
   
   % Update solution
   x = x + dx;
     
elseif m == 0 && l ~= 0
   % Equality constraint
   % Solve the Matrix
   J = [Q -Aeq_in';Aeq_in zeros(l,l)];
   r = [rd;req];
   d = J\(-r);
   dx = d(1:n);
   drho = d(n+1:n+l);
   
   % Update solution
   x = x + eta * dx;
   rho = rho + eta * drho;
   
elseif m ~= 0 && l == 0
   % Inequality constraint
   % Solve the system
   Q_bar = Q + A_in'*(diag(lamda./s))*A_in;
   r_bar = A_in'*((rsl-lamda.*rinq)./s);
   g_bar = -(r_bar+rd);
   L = chol(Q_bar,'lower');
   dx_aff = L'\(L\g_bar);
   ds_aff = -rinq+A_in*dx_aff;
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
   r_bar = A_in'*((rsl-lamda.*rinq)./s);
   g_bar = -(r_bar+rd);
   dx = L'\(L\g_bar);
   ds = -rinq+A_in*dx;
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
  
else
   % Full formulation
   % Calculating the Jacobian of Residue vector r for Newton step
   J = [Q -Aeq_in' -A_in';
        -Aeq_in zeros(l) zeros(l,m);
        -A_in zeros(m,l) -diag(s./lamda)];
   r = -[rd; req; rinq-rsl./lamda];
   
   % Solve the system
%    [L,D,P]=ldl(J);
%    d_aff = P*(L'\(D\(L\(P'*r))));
   d_aff = pinv(J)*r;
   dlamda_aff = d_aff(n+l+1:n+l+m,1);
   ds_aff = -((rsl+s.*dlamda_aff)./lamda);
   
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
   r = -[rd; req; rinq-rsl./lamda];
%    d = P*(L'\(D\(L\(P'*r))));
   d = pinv(J)*r;
   dx = d(1:n,1);
   drho = d(n+1:n+l,1);
   dlamda = d(n+l+1:n+l+m,1);
   ds = -((rsl+s.*dlamda)./lamda);
   
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
   rho = rho + eta*a*drho;
   lamda = lamda + eta*a*dlamda;
   s = s + eta*a*ds;
   
end

end

if ~status
    %Cannot find best solutions
    disp('Cannot find optimized solution!')
end

% Apply the solution
xopt = x;

end