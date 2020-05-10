function x = stateFcn(x,u,md)
Ts = 0.1;
m = 2;
k = 1;
dx = [x(2);
     -1/m*(k*x(1)+x(2)*u(1))+md;
     ];
x = x + Ts*dx; 
end