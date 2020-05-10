function [alpha_opt,flag2] = lineSearch(x,dx,calcP,J,g,h,rho)
%#codegen

%LineSearch
%Arguments
%x:Current solution
%dx:Gradient
%calcP:Merit function function handle
%J:Cost function function handle
%g:Inequality constraints function handle
%h:Equality constraints function handle
%rho:Penalty weight

%ReturnsF
%alpha_opt:Optimum weight
%flag2:Convergence flag

%Line search params
b0 = 10^-2;      %Step width
itermax = 1/b0;

% Initialization
alpha = 0;
alpha_new = 0;
alpha_opt = 1;
flag1 = false;
flag2 = false;

%Enclosure method
for i = 1:itermax
    
    alpha_new = min(alpha,5);
    new = calcP(x+alpha_new*(dx),J,g,h,rho);
    pre = calcP(x+alpha*(dx),J,g,h,rho);
    if new > pre
        amin = b0;
        amax = alpha_new;
        flag1 = true;
        break;
    end
    
    alpha = alpha_new;
    alpha_new = alpha_new + b0;  %Increment
end

% Golden section method
if flag1
    [alpha_opt,flag2] = goldenSection(amin,amax,x,dx,calcP,J,g,h,rho);
end

end

% Golden section method
function [alpha_opt,flag] = goldenSection(amin,amax,x,dx,calcP,J,g,h,rho)

epsilon = 1e-3;
flag = false;

r = (-1+sqrt(5))/2; %1/r = Gold section ratio
 
%Calc internal point
a1 = amin + (1-r)*(amax - amin);
a2 = amin + r*(amax - amin);

%Calc each points
f1 = calcP(x+a1*dx,J,g,h,rho);
f2 = calcP(x+a2*dx,J,g,h,rho);
 
for i = 1:100
    
    %Stop condtion
    if abs(amax - amin) <= epsilon
        alpha_opt = (amin+amax)/2;
        flag = true;
        break
    end
    
    if f1 < f2
        amax = a2;
        a2 = a1;
        f2 = f1;
        a1 = amin + (1-r)*(amax-amin);
        f1 = calcP(x+a1*dx,J,g,h,rho);
    else
        amin = a1;
        a1 = a2;
        f1 = f2;
        a2 = amin + r*(amax-amin);
        f2 = calcP(x+a2*dx,J,g,h,rho);
    end

end

end
