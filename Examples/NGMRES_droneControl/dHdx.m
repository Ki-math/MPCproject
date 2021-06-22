function dHdx = dHdx(X1,X2,X3,X4,X5,X6,X7,X8,X9,X10,X11,X12,lamda1,lamda2,lamda3,lamda4,lamda5,lamda6,lamda7,lamda9,lamda11,m,q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,q11,q12,ref1,ref2,ref3,ref4,ref5,ref6,ref7,ref8,ref9,ref10,ref11,ref12,u1,u2,u3,u4)
%DHDX
%    DHDX = DHDX(X1,X2,X3,X4,X5,X6,X7,X8,X9,X10,X11,X12,LAMDA1,LAMDA2,LAMDA3,LAMDA4,LAMDA5,LAMDA6,LAMDA7,LAMDA9,LAMDA11,M,Q1,Q2,Q3,Q4,Q5,Q6,Q7,Q8,Q9,Q10,Q11,Q12,REF1,REF2,REF3,REF4,REF5,REF6,REF7,REF8,REF9,REF10,REF11,REF12,U1,U2,U3,U4)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    28-Jun-2020 18:36:32

t2 = cos(X7);
t3 = cos(X9);
t4 = cos(X11);
t5 = sin(X7);
t6 = sin(X9);
t7 = sin(X11);
t8 = 1.0./m;
t9 = u1+u2+u3+u4;
dHdx = [q1.*(X1-ref1);lamda1+(q2.*(X2.*2.0-ref2.*2.0))./2.0;q3.*(X3-ref3);lamda3+(q4.*(X4.*2.0-ref4.*2.0))./2.0;q5.*(X5-ref5);lamda5+(q6.*(X6.*2.0-ref6.*2.0))./2.0;q7.*(X7-ref7)-lamda6.*t3.*t5.*t8.*t9+lamda2.*t2.*t3.*t4.*t8.*t9+lamda4.*t2.*t3.*t7.*t8.*t9;lamda7+(q8.*(X8.*2.0-ref8.*2.0))./2.0;q9.*(X9-ref9)+lamda2.*t8.*t9.*(t3.*t7-t4.*t5.*t6)-lamda4.*t8.*t9.*(t3.*t4+t5.*t6.*t7)-lamda6.*t2.*t6.*t8.*t9;lamda9+(q10.*(X10.*2.0-ref10.*2.0))./2.0;q11.*(X11-ref11)+lamda2.*t8.*t9.*(t4.*t6-t3.*t5.*t7)+lamda4.*t8.*t9.*(t6.*t7+t3.*t4.*t5);lamda11+(q12.*(X12.*2.0-ref12.*2.0))./2.0];
