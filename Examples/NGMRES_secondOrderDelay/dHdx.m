function dHdx = dHdx(X1,X2,lamda1,lamda2,q_1,q_2,ref1,ref2,w_n,zeta)
%DHDX
%    DHDX = DHDX(X1,X2,LAMDA1,LAMDA2,Q_1,Q_2,REF1,REF2,W_N,ZETA)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    28-Jun-2020 18:00:59

dHdx = [-lamda2.*w_n.^2+(q_1.*(X1.*2.0-ref1.*2.0))./2.0;lamda1+(q_2.*(X2.*2.0-ref2.*2.0))./2.0-lamda2.*w_n.*zeta.*2.0];