function [A,b] = linInqConstFcn(d,mv0,params,nu)
%#codegen
% Calculating the linear inequality constraint
% Arguments:
%   d:Design variable
%   mv0:Last Input
%   params:Parameter structuer
%   nu:Order for input
% Outputs:
%   A:Inequality Constraint matrix
%   b:Inequality Constraint vector

Hc = params.ControlHorizon;
umin = params.MV_min;
umax = params.MV_max;
dumin = params.dMV_min;
dumax = params.dMV_max;

% Initialization 
scale_mv = params.MV_scalefactor;
A = zeros(4*Hc*nu,length(d));
b = zeros(4*Hc*nu,1);

% Input magnitude constraint 
Au = zeros(Hc*nu);
bu_max = zeros(Hc*nu,1);
bu_min = zeros(Hc*nu,1);
Au = kron(eye(Hc),eye(nu));
bu_max = kron(ones(Hc,1),umax./scale_mv); % Convert to demension less
bu_min = kron(ones(Hc,1),-umin./scale_mv);

% Input rate constraint 
Adu = zeros(Hc*nu);
bdu_max = zeros(Hc*nu,1);
bdu_min = zeros(Hc*nu,1);
idx_row = 1:nu;
idx_col = 1:nu;
for i = 1:Hc
    if i == 1
        Adu(idx_row,idx_col) = eye(nu);
        bdu_max(idx_row,1) = (dumax + mv0)./scale_mv;   % Convert to demension less
        bdu_min(idx_row,1) = -(dumin + mv0)./scale_mv;
        idx_row = idx_row + nu;
    else
        Adu(idx_row,idx_col) = -eye(nu);
        idx_col = idx_col + nu;
        Adu(idx_row,idx_col) = eye(nu);
        bdu_max(idx_row,1) = dumax./scale_mv;
        bdu_min(idx_row,1) = -dumin./scale_mv;
        idx_row = idx_row + nu;
    end
end

A = [Au zeros(Hc*nu,1);
    -Au zeros(Hc*nu,1);
    Adu zeros(Hc*nu,1);
    -Adu zeros(Hc*nu,1)];
b = [bu_max;bu_min;bdu_max;bdu_min];

end