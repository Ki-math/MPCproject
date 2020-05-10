function [xopt,lamda,mu,iA1,status] = activeSet(x,Q,c,A,b,Aeq,beq,iA0,iterMax)

%Active Set Algorithm
%
%Arguments
%x:Initial guess of design variable
%Q:Weight of quadratic term (hessian)
%c:Coefficient of 1st order term (jacobian)
%A:Inequality coefficient of Ax<=b
%b:Inequality constant of Ax<=b
%Aeq:Equality coefficient of Ax=b
%beq:Equality Constant of Ax=b
%iA0:Initial guess of Active Constraints 1st colmun:flag 2nd colmun:Identify of constraints (1:Inequality 2:Equality)
%iterMax:Max iteration number
%
%Output
%xopt:Resolution
%lamda:Lagrange variable of inequality constaints
%mu:Lagrange variable of equality constaints
%iA1:Active set which structure is same with iA0 
%status:Optimization flag 1:feasible 0:infeasible
%
%Cost function form
% J = 0.5*x'*Q*x + c'*x
% Aeq*x = beq
% A*x<b

%Order of variables
l = length(x);   %Design variable
m = size(A,1);   %Inequality constraints
o = size(Aeq,1); %Equality constraints

%Initialization
xopt = zeros(l,1);
lamda = zeros(m,1);
mu = zeros(o,1);
status = false;
iA1 = zeros(size(iA0,1),2,'like',iA0);
xhat = zeros(l,1);
min_index = 0;

%tollerance
tol = 1e-6;

%Flag judgement of KKT
flag = true;

%Order of initial guess of active set
if isempty(iA0)
    iAn = 0;
else
    iAn = m+o;
    iA1 = iA0;
end

if iAn == 0
    %Cold start
    if m ~= 0
        for i = 1:m
           diff = A(i,1:l)*x - b(i);
           if abs(diff) <= tol
               iA1(i,1) = 1;   %ActiveSet Flag True:1
           else
               iA1(i,1) = 0;   %ActiveSet Flag False:0
           end
           iA1(i,2) = 1;       %Inequality constarint id:1
        end
    end
    
    %Equality constraints must be included in Active set
    if o ~= 0
       for i = m+1:m+o
          iA1(i,1) = 1;
          iA1(i,2) = 2;   %Equality constraint id=2
       end
    end
end

if m > 0 && o > 0
    AA = [A;Aeq];
    bb = [b;beq];
elseif m > 0 && o == 0
    AA = A;
    bb = b;
else
    AA = Aeq;
    bb = beq;
end

for j = 1:iterMax
        %'n' is the number for Order of Active Set
        n = 0;
        for i = 1:m+o
            if iA1(i,1) > 0
                n = n + 1;
            end
        end
%         assert (n <= 500);
        Ai = zeros(n,l);
        bi = zeros(n,3);
        
        if n ~= 0
            idx = 1;

            for i = 1:(m+o)
                if iA1(i,1) == 1 && iA1(i,2) == 1
                   %Inequality
                   Ai(idx,1:l) = AA(i,1:l);
                   bi(idx,1) = bb(i);
                   bi(idx,2) = 1;       %Identifier of constraint
                   bi(idx,3) = i;       %Index of constraint
                   idx = idx + 1;
                elseif iA1(i,2) == 2
                   %Equality
                   Ai(idx,1:l) = AA(i,1:l);
                   bi(idx,1) = bb(i);
                   bi(idx,2) = 2;       %Identifier of constraint
                   bi(idx,3) = i;       %Index of constraint
                   idx = idx + 1;
                end
                if idx > n
                    break;
                end
            end
            
            %Solve
            left = [Q Ai';
                    Ai zeros(n)];
            right = [-c;bi(1:n,1)];
            sol_hat = pinv(left)*right;
        else
            %If list is null, next is done
            left = Q;
            right = -c;
            sol_hat = left\right;
        end
        
        %Extract solution
        xhat = sol_hat(1:l,1);
        
        %Check solution,whether or not they does not violate all constarints
        for i = 1:m+o
            diff = AA(i,1:l)*xhat - bb(i);
            if diff > tol
                %If the solutions has violation of constrain even just one,
                %the flag sets to false and loop break
                flag = false;
                break;
            end
        end
        
        if ~flag
            %Modifying solution due to violating un-active set
            t = inf(m+o,1);
            for i = 1:m+o
                if iA1(i,1) == 0 && iA1(i,2) == 1
                    temp = AA(i,1:l)*(xhat - x);
                    if  temp > 0
                        t(i) = -(AA(i,1:l)*x - bb(i))/temp;
                    end
                end
            end
            
            [mint,index] = min(t);
            alpha = max(min(1,mint),0); %0<=alpha<=1
            x = x + alpha*(xhat - x);
            
            %Enable
            iA1(index,1) = 1;
            
            %Change the flag to true
            flag = true;
        else
            %Check KKT condtion for feasible solution
            if n ~= 0
                %If problem has active set
                lmd = inf(n,1);
                lmd = sol_hat(l+1:l+n);
                n_inq = 0;
                n_eq = 0;
                for i = 1:n
                    if bi(i,2) == 1
                        n_inq = n_inq +1;
                    end
                end
                for i = 1:n
                    if bi(i,2) == 2
                        n_eq = n_eq + 1;
                    end
                end

                idx1 = 1;
                idx2 = 1;
                lmd_ineq = zeros(n_inq,1);
                lmd_eq = zeros(n_eq,1);
                for i = 1:n
                    if bi(i,2) == 1
                        %Extract lagrange variable of inequality constraints
                        lmd_ineq(idx1,1) = lmd(i);
                        idx1 = idx1 + 1;
                    else
                        %Extract lagrange variable of equality constraints
                        lmd_eq(idx2,1) = lmd(i);
                        idx2 = idx2 + 1;
                    end
                end
                
                %Extract positive value among variables
                chk = 0;
                for i = 1:n_inq
                    if lmd_ineq(i) >= 0
                        chk = chk + 1;
                    end
                end
                
                %Check convergence
                if n_inq == 0   
                    %no inequality constraints
                    xopt = xhat;
                    lamda(1:n_inq,1) = lmd_ineq;
                    mu(1:n_eq,1) = lmd_eq;
                    status = true;
                    break;
                elseif chk == n_inq
                    %Check all lagrange variable of inequality constraints are positive
                    %Optimum solution
                    xopt = xhat;
                    lamda(1:n_inq,1) = lmd_ineq;
                    mu(1:n_eq,1) = lmd_eq;
                    status = true;
                    break;
                else
                    %Remove an inequality constarinat according to minimum lagrange variable from active set
                    [~,min_index] = min(lmd);
                    if min_index ~= 0
                        %Change flag to unactive
                        iA1(bi(min_index,3),1) = 0;
                    end
                    flag = true;    %Change the flag to true
                end
            else
                %Reach optimum solution with no active set
                xopt = xhat;
                lamda = zeros(m,1);
                mu = zeros(o,1);
                status = true;
                break;
            end
        end
end

if ~status
    %Cannot find any solutions
    %Reset active set
    if m ~= 0
        iA1(1:m,1) = zeros(m,1);
        iA1(1:m,2) = ones(m,1);
    end
    if o ~= 0
        iA1(m+1:m+o,1) = zeros(o,1);
        iA1(m+1:m+o,2) = 2*ones(o,1);
    end
%     disp('Cannot find solution!')
end

end