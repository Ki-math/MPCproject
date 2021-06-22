function dHdu = dHdu(Jx,Jy,Jz,X7,X9,X11,k,l,lamda2,lamda4,lamda6,lamda8,lamda10,lamda12,m,r1,r2,r3,r4,u1,u2,u3,u4)
%DHDU
%    DHDU = DHDU(JX,JY,JZ,X7,X9,X11,K,L,LAMDA2,LAMDA4,LAMDA6,LAMDA8,LAMDA10,LAMDA12,M,R1,R2,R3,R4,U1,U2,U3,U4)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    28-Jun-2020 18:36:33

t2 = cos(X7);
t3 = cos(X9);
t4 = cos(X11);
t5 = sin(X7);
t6 = sin(X9);
t7 = sin(X11);
t8 = 1.0./Jx;
t9 = 1.0./Jy;
t10 = 1.0./Jz;
t11 = 1.0./m;
t12 = t4.*t6;
t13 = t6.*t7;
t14 = t3.*t4.*t5;
t15 = t3.*t5.*t7;
t16 = k.*lamda12.*t10;
t17 = l.*lamda8.*t8;
t18 = l.*lamda10.*t9;
t21 = lamda6.*t2.*t3.*t11;
t19 = -t15;
t20 = -t16;
t22 = t13+t14;
t23 = t12+t19;
t24 = lamda2.*t11.*t22;
t25 = lamda4.*t11.*t23;
t26 = -t25;
dHdu = [t16+t17+t21+t24+t26+r1.*u1;t16-t17+t21+t24+t26+r2.*u2;t18+t20+t21+t24+t26+r3.*u3;-t18+t20+t21+t24+t26+r4.*u4];