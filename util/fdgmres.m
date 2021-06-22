function [du, error, total_iters] = fdgmres(f0,x0,u0,d,ref,du0,ts,params,model,dPhidx,dHdx,dHdu)
%#codegen
% Forward Differential Generalized minal residual method
% This is based a code by C. T. Kelley.
% GMRES linear equation solver for use in Newton-GMRES solver
% 
% Arguments:  
%   f0:function at current point
%   x0:Current state
%   u0:Current input (Solution vector)
%   d:Disturbance
%   ref:Reference input
%   du0:Initial estimation for rate of input vector
%   ts:Sampling time
%   params:Parameter structure
%   model:Function handle for statefunction
%   dPhidx:Function handle for jacobian of the terminate function
%   dHdx:Function handle for jacobian of the hamiltonian function
%   dHdu:Function handle for jacobian of the hamiltonian function
% 
% Output: 
%   du:solution vector
%   error:vector of residual norms for the history of the iteration
%   total_iters:number of iterations
%
% Requires givapp.m, dirder.m 

% initialization
errtol = 1e-6;
kmax = params.InnerIterationMax;
reorth = 1;

% right side of linear equation for the step is -f0 if the
% default initial iterate is used
b = -f0;
n = length(b);

% Use zero vector as initial iterate for Newton step unless
% the calling routine has a better idea (useful for GMRES(m)).
du = zeros(n,1);
r = b;
if ~(isempty(du0))
    du = du0;
    r = -dirder(f0,x0,u0,d,ref,du0,ts,params,model,dPhidx,dHdx,dHdu) - f0;
end
h = zeros(kmax);
v = zeros(n,kmax);
c = zeros(kmax+1,1);
s = zeros(kmax+1,1);
rho = norm(r);
g = rho * eye(kmax+1,1);
errtol = errtol * norm(b);
error=[];

% test for termination on entry
error = [error,rho];
total_iters = 0;
if(rho < errtol) 
%   disp(' early termination ')
    return
end
v(:,1) = r/rho;
beta = rho;
k = 0;

% GMRES iteration
while((rho > errtol) && (k < kmax-1))
    k = k+1;
    
    % call directional derivative function
    v(:,k+1) = dirder(f0,x0,u0,d,ref,v(1:n,k),ts,params,model,dPhidx,dHdx,dHdu);
    normav = norm(v(:,k+1));

    % Modified Gram-Schmidt
    for j = 1:k
        h(j,k) = v(:,j)'*v(:,k+1);
        v(:,k+1) = v(:,k+1)-h(j,k)*v(:,j);
    end
    
    h(k+1,k) = norm(v(:,k+1));
    normav2 = h(k+1,k);
    
    % reorthogonalize?
    if  (reorth == 1 && normav+1e-1*normav2 == normav)
        for j = 1:k
            hr = v(:,j)'*v(:,k+1);
            h(j,k) = h(j,k) + hr;
            v(:,k+1) = v(:,k+1) - hr*v(:,j);
        end
        h(k+1,k) = norm(v(:,k+1));
    end
    
    % watch out for happy breakdown
    if(h(k+1,k) ~= 0)
        v(:,k+1) = v(:,k+1)/h(k+1,k);
    end
    
    % Form and store the information for the new Givens rotation
    if k > 1
        h(1:k,k) = givapp(c(1:k-1),s(1:k-1),h(1:k,k),k-1);
    end
    
    % Don't divide by zero if solution has been found
    nu = norm(h(k:k+1,k));
    if nu ~= 0
    %        c(k)=h(k,k)/nu;
        c(k) = conj(h(k,k)/nu);
        s(k) = -h(k+1,k)/nu;
        h(k,k) = c(k)*h(k,k) - s(k)*h(k+1,k);
        h(k+1,k) = 0;
        g(k:k+1) = givapp(c(k),s(k),g(k:k+1),1);
    end
    
    % Update the residual norm
    rho = abs(g(k+1));
    error = rho;
    
end

% At this point either k > kmax or rho < errtol.
% It's time to compute x and leave.
y = h(1:k,1:k)\g(1:k);
du = du + v(1:n,1:k)*y;
total_iters = k;
end
%EOF