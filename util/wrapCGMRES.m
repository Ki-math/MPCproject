function [mv,status] = wrapCGMRES(x0,ref,md,params,nmv,Ts)
%#codegen
% Wrapper function for cgmresfcn  
% 
% Arguments
% x0:Intial State
% ref:Refernce value of state
% md:Measurement Disturbance
% params:Parameter Structure
% nmv:Number of mv
% 
% Outputs
% mv:Manipulated Variable
% status:Optimization Flag True or False

% Permanent variables
persistent flag;
persistent u;
persistent du;
persistent T;

% Initialization
len_u = length(params.initialConditionInputs);

if isempty(flag)
    flag = false;
end
if isempty(du)
    du = zeros(len_u*params.divideNumber,1);
end
if isempty(u)
    u = zeros(len_u*params.divideNumber,1);
end
if isempty(T)
    T = 0;
end
if ~flag
    du = repmat(params.initialConditionInputs,params.divideNumber,1);
    u = du;
    flag = true;
end

% Solve optimization problem by CGMRES method
[duopt,status] = cgmresfcn(x0,u,du,md,ref,T,params);
if status
    du = duopt;
    u = u + du*Ts;                          % Integrating mv
end

% Output mv value
mv = u(1:nmv);

% Increasing predicition time as first order delay
T = params.T_a*T + params.T_b;    
end