function xdot = model(x,u,md)
%���䃂�f�����L�q
xdot = [x(2);
        -2*0.1*x(2)-x(1)+u(1)];
end