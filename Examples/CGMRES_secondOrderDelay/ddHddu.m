function y = ddHddu(x,lamda,u,params)
    %u(1)�F�������
    %u(2)�F�X���b�N�ϐ�
    %u(3)�F���O�����W���搔
    y = [params.R+2*u(3) 0 2*u(1);
         0 2*u(3) 2*u(2);
         2*u(1) 2*u(2) 0];
end