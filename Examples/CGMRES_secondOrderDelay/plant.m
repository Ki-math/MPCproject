function xdot = plant(x,u)
%�v�����g���f�����L�q
xdot = [x(2);
        -2*0.1*x(2)-x(1)+u];
end