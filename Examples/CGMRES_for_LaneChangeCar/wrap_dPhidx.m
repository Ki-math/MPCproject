function out = wrap_dPhidx(x,ref,params)
% #codegen
% Wapper function of dPhidx
% state_names = {'px', 'py', 'theta', 'r', 'beta'};
% normal_input_names = 'delta';
px = x(1);
py = x(2);
theta = x(3);
r = x(4);
beta = x(5);
ref1 = ref(1);
ref2 = ref(2);
ref3 = ref(3);
ref4 = ref(4);
ref5 = ref(5);
q_f1 = params.Qf(1);
q_f2 = params.Qf(2);
q_f3 = params.Qf(3);
q_f4 = params.Qf(4);
q_f5 = params.Qf(5);

out = dPHIdx(beta,px,py,q_f1,q_f2,q_f3,q_f4,q_f5,r,ref1,ref2,ref3,ref4,ref5,theta);

end