function out = wrap_dPhidx(x,ref,params)
% #codegen
% Wapper function of dPhidx
X1 = x(1);
X2 = x(2);
ref1 = ref(1);
ref2 = ref(2);
q_f1 = params.Qf(1);
q_f2 = params.Qf(2);

out = dPHIdx(X1,X2,q_f1,q_f2,ref1,ref2);

end