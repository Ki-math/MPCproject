function out = wrap_ddHddu(x,lmd,u,d,ref,params)
% #codegen
% Wrapper function of ddHddu
r1 = params.R(1);
r2 = params.R(2);
r3 = params.R(3);
r4 = params.R(4);
out = ddHddu(r1,r2,r3,r4);
end