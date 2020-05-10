function [x_stop,z_stop,s_stop,k] = pc_QPsimplev1(x,z,s,G,g,A,b)
%----------------------------------------------------------
% pcQP_simplev1.m
%
% This function solves a QP problem of the form
%
% min 0.5xÅfGx + gÅfx
% s.t. AÅfx >= b
%
% using the Predictor-Corrector (PC) method .
%
% Input :
% x : starting point for the vector x ( n x 1 vector )
% z : starting point for the vector x (nA x 1 vector )
% s : starting point for the slack-vector x (nA x 1 vector )
% G: Hessian ( n x n matrix )
% g : ( n x 1 vector )
% A: left hand side of inequality constraints ( n x nA matrix )
% b : right hand side of inequality constraints (nA x 1 vector )
% where nC and nA are the numbers of inequality and equality
% constraints.
% Output :
% x_stop : solution x
% z_stop
% s_stop
% k : number of iterations used
%
% Thomas Reslow KrÅNuth , s021898
%----------------------------------------------------------
eta = 0.95;
%residuals are computed
[mA, nA] = size(A);
e = ones (nA, 1);
rL = G * x + g - A * z;
rs = s - A'* x + b;
rsz = s .* z;
mu = sum (z .* s) / nA;
%k : number of iterations , epsilon : tolerances
k = 0;
maxk = 200;
eps_L = 1e-16; eps_s = 1e-16; eps_mu = 1e-16;

while (k<=maxk && norm(rL) >= eps_L && norm( rs ) >= eps_s && abs(mu) >= eps_mu )
    %Solve system with a Newton-like method/ Factorizarion
    G_bar = G + A * ( diag (z ./ s)) * A';
    r_bar = A * ((rsz - z .* rs) ./ s);
    g_bar = -(rL + r_bar);
    L = chol(G_bar , 'lower');
    dx_a = L' \ (L \ g_bar);
    ds_a = -rs + A' * dx_a;
    dz_a = -(rsz + z .* ds_a) ./ s;
    
    %Compute alphaaff
    alpha_a = 1;
    idx_z = find( dz_a < 0);
    if (isempty(idx_z) == 0)
        alpha_a = min(alpha_a , min(-z(idx_z) ./ dz_a (idx_z)));
    end
    
    idx_s = find(ds_a < 0);
    if (isempty (idx_s) == 0)
        alpha_a = min(alpha_a , min(-s(idx_s) ./ ds_a(idx_s)));
    end
    
    %Compute the affineduality gap
    mu_a = ((z + alpha_a * dz_a)' * (s + alpha_a * ds_a)) / nA;
    %Compute the centering parameter
    sigma = ( mu_a/mu) ^ 3;
    %Solve system
    rsz = rsz + ds_a .* dz_a - sigma * mu * e;
    r_bar = A * (( rsz - z .* rs) ./ s);
    g_bar = -(rL + r_bar);
    dx = L' \ (L \ g_bar);
    ds = -rs + A' * dx;
    dz = -(rsz + z .* ds) ./ s;
    
    %Compute alpha
    alpha = 1;
    idx_z = find(dz < 0);
    if (isempty(idx_z) == 0)
        alpha = min(alpha, min(-z(idx_z) ./ dz(idx_z)));
    end
    idx_s = find(ds < 0);
    if (isempty(idx_s) == 0)
        alpha = min(alpha, min(-s(idx_s) ./ ds(idx_s)));
    end
    
    %Update x , z , s
    x = x + eta * alpha * dx;
    z = z + eta * alpha * dz;
    s = s + eta * alpha * ds;
    k = k + 1;
    
    %Update rhs and mu
    rL = G * x + g - A * z;
    rs = s - A' * x + b;
    rsz = s .* z;
    mu = sum(z .* s) / nA;
end

x_stop = x;
z_stop = z;
s_stop = s;

end