function y = dHdu(x,lamda,u,d,r,params)
    %u(1)�F�������
    %u(2)�F�X���b�N�ϐ�
    %u(3)�F���O�����W���搔
    y = [params.R*(u(1))+lamda(2)+2*u(3)*u(1);
         -0.05+2*u(3)*u(2);
         u(1)^2+u(2)^2-params.MV_max^2];
         
end