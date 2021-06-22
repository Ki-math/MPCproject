function out = wrap_dHdu(x,lmd,u,d,ref,params)
% #codegen
% Wrapper function of dHdx
R = params.R(1);
lamda2 = lmd(2);
w_n = 1;
rho = u(3);
s = u(2);
u = u(1);
u_sat = params.MV_max;
w_s = 0.01;
out = dHdu(R,lamda2,rho,s,u,u_sat,w_n,w_s);
end