function out = wrap_ddHddu(x,lmd,u,d,ref,params)
% #codegen
% Wrapper function of ddHddu
% state_names = {'px', 'py', 'theta', 'r', 'beta', 'V'};
% normal_input_names = {'delta', 'a'};
delta = u(1);
s = u(2);          % Dummy input (slack) for the 1st input constraint
rho = u(3);        % Lagrange variable for the the 1st input constraint
R1 = params.R;
out = ddHddu(R1,delta,rho,s);
end