function out = wrap_statefcn(x,u,d)
% #codegen
X2 = x(2);
X4 = x(4);
X6 = x(6);
X7 = x(7);
X8 = x(8);
X9 = x(9);
X10 = x(10);
X11 = x(11);
X12 = x(12);
u1 = u(1);
u2 = u(2);
u3 = u(3);
u4 = u(4);
g = 9.81;
m = 1.4;
l = 0.2;
Jx = 0.03;
Jy = 0.03;
Jz = 0.04;
k = 4;

out = statefcn(Jx,Jy,Jz,X2,X4,X6,X7,X8,X9,X10,X11,X12,g,k,l,m,u1,u2,u3,u4);

end