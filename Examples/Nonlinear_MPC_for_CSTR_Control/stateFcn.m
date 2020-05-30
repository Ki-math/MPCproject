function xd = stateFcn(x,u,md)
CAi = 10;
Ts = 0.1;
T = x(2);
CA = x(1);
Ti = md;
Tc = u;
dxdt = nonlinearCSTR(T,CA,CAi,Ti,Tc);
xd = x + Ts*dxdt;
end