function y = ddHddu(x,lamda,u,params)
    %u(1)：制御入力
    %u(2)：スラック変数
    %u(3)：ラグランジュ乗数
    y = [params.R+2*u(3) 0 2*u(1);
         0 2*u(3) 2*u(2);
         2*u(1) 2*u(2) 0];
end