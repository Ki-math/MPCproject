function y = dHdu(x,lamda,u,d,r,params)
    %u(1)：制御入力
    %u(2)：スラック変数
    %u(3)：ラグランジュ乗数
    y = [params.R*(u(1))+lamda(2)+2*u(3)*u(1);
         -0.05+2*u(3)*u(2);
         u(1)^2+u(2)^2-params.MV_max^2];
         
end