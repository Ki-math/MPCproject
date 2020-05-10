function [xopt,iAopt,flagOpt] = sqp(x,J,g,h,iA0,itermax)
%#codegen
%Sequential quadratic programming

%Arguments
%x:Initial design variable
%J:Cost function
%f:Jacobian of cost function
%g:Inequality constraints 
%gd:Gradient of inequality constraints g + gd * dx <= 0 (gd * dx <= -g)
%h:Equality constraints
%hd:Gradient of equality constraints h + hd * dx = 0 (hd * dx = -h)
%iA0:Active Constraints order: 1st index 2nd id (1(inequality) or 2(equality))

%Output
%xopt:Resolution
%iA:Active constraint order: 1st index 2nd id (1(inequality) or 2(equality))
%flagOpt:Optimized flag true or false

%Cost function form (2nd order approximation)
% 0.5*dx'*H*dx + f'*dx
% H:Hessian
% f:Jacobian

%Definition of Jacobian
% If f is scalar function and x is x = [x1 x2 ... xn]' then
% df/dx = [dfdx1; dfdx2; ... dfdxn;].

% If f is vector function f = [f1 f2 .... fn] and x is x = [x1 x2 ... xn]' then
% df/dx = [df1dx1 df2dx1 ... dfndx1;
%          df1dx2 df2dx2 ... dfndx2;
%           :       :           :  ;
%          d1ndxn df2dxn ... dfndxn].

% Optimization parameters
tollerance = 1e-6;

% Params for liner search
rho = 1e4;
guzai = 0.9;
tau = 0.9;

% Initialization
n = length(x);          % Dimension of design varibale

if ~isempty(g)          % Dimension of inequality constraints
    nineq = size(g(x),1);  
else
    nineq = 0;
end

if ~isempty(h)          % Dimension of equality constraints
    neq = size(h(x),1);   
else
    neq = 0;
end

ntotal = nineq + neq;   % Total number of constraints
iA = iA0;               % Active set during optimization

dx = zeros(n,1);        % Pertubation of design variable
H = eye(n);             % Initial estimate of Hessian
xopt = zeros(n,1);      % Optimized variable
iAopt = zeros(ntotal,2);% Active set after optimization
flagOpt = false;        % Optimize flag

for i = 1:itermax

    %linear approximation of each constraints
    %Inequality constraints A*x<=b <-> gd'*dx <= -g
    if ~isempty(g)
        A = calcJacobian(x,g)';
        b = -g(x);
    else
        A = [];
        b = [];
    end

    %Equality constraints Aeq*x<=beq <-> hd'*dx = -h
    if ~isempty(h)
        Aeq = calcJacobian(x,h)';
        beq = -h(x);
    else
        Aeq = [];
        beq = [];
    end

    %Jacobian of cost function
    c = calcJacobian(x,J);

    %Solve subset optimal problem by active set
    % Initialization
    lamda = zeros(size(A,1),1);
    mu = zeros(size(Aeq,1),1);
    iA1 = zeros(ntotal,2);
    flagAC = false;
%     [dx,lamda,mu,iA1,flagAC] = activeSet(dx,H,c,A,b,Aeq,beq,iA0,itermax);
    opts = optimoptions('quadprog','Display','off');
    [dx,~,flagQC] = quadprog(H,c,A,b,Aeq,beq,[],[],[],opts);
    if flagQC == 1
        flagAC = true;
    end

    if flagAC
        % Convergence judgement
        if norm(dx) <= tollerance
            xopt = x;
            iAopt = iA1;
            flagOpt = true;
            break;
        end

        % Line search by Armijo condition 
        flag2 = false;      %Convergence flag for line search

%         beta = 1;
%         alpha = 0;
%         Pleft = calcP(x+beta*dx,J,g,h,rho);
%         for ii = 1:1000
%             P = calcP(x,J,g,h,rho);
%             Pl = calcPl(x,dx,J,g,h,rho);
%             deltaP = Pl - P;
%             Pright = P + guzai * beta * deltaP;
% 
%             if Pleft <= Pright
%                 alpha = beta;
%                 flag2 = true;
%                 break;
%             else
%                 beta = tau * beta;
%             end
%         end
% 
%         [alpha,flag2] = lineSearch(x,dx,@calcP,J,g,h,rho);
        
        xpre = x;
        if flag2
            x = x + alpha * dx;
        else
            x = x + dx;
        end

        %Estimate hessian by calculating modified BFGS method
        s = x - xpre;
        if ~isempty(g) && ~isempty(lamda)
            inequalityTerm_cur = calcJacobian(x,g)*lamda;
            inequalityTerm_pre = calcJacobian(xpre,g)*lamda;
        else
            inequalityTerm_cur = zeros(n,1);
            inequalityTerm_pre = zeros(n,1);
        end

        if ~isempty(h) && ~isempty(mu)
            equalityTerm_cur = calcJacobian(x,h)*mu;
            equalityTerm_pre = calcJacobian(xpre,h)*mu;
        else
            equalityTerm_cur = zeros(n,1);
            equalityTerm_pre = zeros(n,1);
        end

        dLdx_new = calcJacobian(x,J) + inequalityTerm_cur + equalityTerm_cur;
        dLdx_old = calcJacobian(xpre,J) + inequalityTerm_pre + equalityTerm_pre;
        q = dLdx_new - dLdx_old;

        sq = s'*q;
        sHs = s'*H*s;
        if sq >= 0.2*sHs
            psi = 1;
        else
            psi = 0.8*sHs/(s'*(H*s-q));
        end
        qhat = psi*q + (1-psi)*H*s;
        H = H - H*s*(H*s)'/sHs + qhat*qhat'/(s'*qhat);

        %Update active set
        iA = iA1;
    else
        % Not find feasible solution by Active Set
        break;
    end
end

end
