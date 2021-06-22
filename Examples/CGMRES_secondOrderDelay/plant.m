function xdot = plant(x,u)
%プラントモデルを記述
xdot = [x(2);
        -2*0.1*x(2)-x(1)+u];
end