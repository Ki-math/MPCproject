function out = wrap_dHdx(x,lmd,u,d,ref,params)
% #codegen
% Wrapper function of dHdx
X1 = x(1);
X2 = x(2);
ref1 = ref(1);
ref2 = ref(2);
q_1 = params.Q(1);
q_2 = params.Q(2);
lamda1 = lmd(1);
lamda2 = lmd(2);
w_n = 1;
zeta = 0.7;

out = dHdx(X1,X2,lamda1,lamda2,q_1,q_2,ref1,ref2,w_n,zeta);
end