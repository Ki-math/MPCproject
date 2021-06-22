function xdot = statefcn(x,u,d)
zeta = 0.7;
wn = 1;
xdot = [x(2);
        -wn^2*x(1)-2*zeta*wn*x(2)+wn^2*u(1)];
end