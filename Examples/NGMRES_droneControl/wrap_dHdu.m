function out = wrap_dHdu(x,lmd,u,d,ref,params)
% #codegen
% Wrapper function of dHdx
X7 = x(7);
X9 = x(9);
X11 = x(11);
u1 = u(1);
u2 = u(2);
u3 = u(3);
u4 = u(4);
r1 = params.R(1);
r2 = params.R(2);
r3 = params.R(3);
r4 = params.R(4);
lamda2 = lmd(2);
lamda4 = lmd(4);
lamda6 = lmd(6);
lamda8 = lmd(8);
lamda10 = lmd(10);
lamda12 = lmd(12);
g = 9.81;
m = 1.4;
l = 0.2;
Jx = 0.03;
Jy = 0.03;
Jz = 0.04;
k = 4;
out = dHdu(Jx,Jy,Jz,X7,X9,X11,k,l,lamda2,lamda4,lamda6,lamda8,lamda10,lamda12,m,r1,r2,r3,r4,u1,u2,u3,u4);
end