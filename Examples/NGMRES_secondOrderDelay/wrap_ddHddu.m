function out = wrap_ddHddu(x,lmd,u,d,ref,params)
% #codegen
% Wrapper function of ddHddu
R = params.R;
rho = u(3);
s = u(2);
u = u(1);
out = ddHddu(R,rho,s,u);
end