function z = dirder(f0,x0,u0,d,ref,du0,ts,params,model,dPhidx,dHdx,dHdu)
%#codegen
% This is based a code by C. T. Kelley at:
% Finite difference directional derivative
% Approximate f'(x)*w
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
% Outputs:
%   z:Difference derivative approximation about f'(x)*w

% scale the step
if norm(du0) == 0
    n = length(du0);
    z = zeros(n,1);
    return
end
% epsnew = params.h/norm(du0);
% if norm(u0) > 0
%     epsnew = epsnew*norm(x);
% end
epsnew=1.d-4;

f1 = calcDiff(x0,u0+epsnew*du0,d,ref,ts,params,model,dPhidx,dHdx,dHdu);
z = (f1 - f0)/epsnew;
end
%EOF