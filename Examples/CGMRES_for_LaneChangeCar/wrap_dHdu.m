function out = wrap_dHdu(x,lmd,u,d,ref,params)
% #codegen
% Wrapper function of dHdx
% state_names = {'px', 'py', 'theta', 'r', 'beta'};
% normal_input_names = {'delta', 'a'};
V = 15;
m = 2000;
l_f = 1.4;
I = 4000;
K_f = 12e3;
delta = u(1);
s = u(2);          % Dummy input (slack) for the 1st input constraint
rho = u(3);        % Lagrange variable for the the 1st input constraint
lamda4 = lmd(4);
lamda5 = lmd(5);
R1 = params.R(1);
w_s = 0.01;        % Weight for the dummy input (slack)
delta_max = params.MV_max(1);

out = dHdu(I,K_f,R1,V,delta,delta_max,l_f,lamda4,lamda5,m,rho,s,w_s);
end