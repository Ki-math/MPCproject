function out = wrap_dHdx(x,lmd,u,d,ref,params)
% #codegen
% Wrapper function of dHdx
% state_names = {'px', 'py', 'theta', 'r', 'beta'};
% normal_input_names = 'delta';
px = x(1);
py = x(2);
theta = x(3);
r = x(4);
beta = x(5);
V = 15;
m = 2000;
l_f = 1.4;
l_r = 1.6;
I = 4000;
K_f = 12e3;
K_r = 11e3;
ref1 = ref(1);
ref2 = ref(2);
ref3 = ref(3);
ref4 = ref(4);
ref5 = ref(5);
q_1 = params.Q(1);
q_2 = params.Q(2);
q_3 = params.Q(3);
q_4 = params.Q(4);
q_5 = params.Q(5);
lamda1 = lmd(1);
lamda2 = lmd(2);
lamda3 = lmd(3);
lamda4 = lmd(4);
lamda5 = lmd(5);

% Constraints
W = 5;          % Width of road [m]
radius = 3;     % Radius of obstacle [m]

% Penalty weight for barrier function
p = 1e7;

out = dHdx(I,K_f,K_r,V,W,beta,l_f,l_r,lamda1,lamda2,lamda3,lamda4,lamda5,m,p,px,py,q_1,q_2,q_3,q_4,q_5,r,radius,ref1,ref2,ref3,ref4,ref5,theta);
end