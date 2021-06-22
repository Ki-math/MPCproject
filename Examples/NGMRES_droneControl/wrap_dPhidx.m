function out = wrap_dPhidx(x,ref,params)
% #codegen
% Wapper function of dPhidx
X1 = x(1);
X2 = x(2);
X3 = x(3);
X4 = x(4);
X5 = x(5);
X6 = x(6);
X7 = x(7);
X8 = x(8);
X9 = x(9);
X10 = x(10);
X11 = x(11);
X12 = x(12);
ref1 = ref(1);
ref2 = ref(2);
ref3 = ref(3);
ref4 = ref(4);
ref5 = ref(5);
ref6 = ref(6);
ref7 = ref(7);
ref8 = ref(8);
ref9 = ref(9);
ref10 = ref(10);
ref11 = ref(11);
ref12 = ref(12);
q_f1 = params.Qf(1);
q_f2 = params.Qf(2);
q_f3 = params.Qf(3);
q_f4 = params.Qf(4);
q_f5 = params.Qf(5);
q_f6 = params.Qf(6);
q_f7 = params.Qf(7);
q_f8 = params.Qf(8);
q_f9 = params.Qf(9);
q_f10 = params.Qf(10);
q_f11 = params.Qf(11);
q_f12 = params.Qf(12);

out = dPHIdx(X1,X2,X3,X4,X5,X6,X7,X8,X9,X10,X11,X12,q_f1,q_f2,q_f3,q_f4,q_f5,q_f6,q_f7,q_f8,q_f9,q_f10,q_f11,q_f12,ref1,ref2,ref3,ref4,ref5,ref6,ref7,ref8,ref9,ref10,ref11,ref12);

end