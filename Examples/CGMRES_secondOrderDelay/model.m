function xdot = model(x,u,md)
%制御モデルを記述
xdot = [x(2);
        -2*0.1*x(2)-x(1)+u(1)];
end