addpath('C:\Users\kohei\Desktop\myWork\MPC\MPC\util');
clc
clear

% パラメータ
zeta = 0.7;
wn = 1;

% 初期状態量
x0 = [0;0];

% サンプリング時間 
Ts = 1e-3;

params.Qf = [10;1];
params.Q = [10;1];
params.R = 0.1;
params.PredictiveTime = 1;
params.divideNumber = 10;
params.MV_max = 10;                   % Inputs Constraints
params.stateFunction = 'statefcn';
params.jacobianTerminalFunction = 'wrap_dPhidx';
params.jacobianHamiltonianState = 'wrap_dHdx';
params.jacobianHamiltonianInput = 'wrap_dHdu';
params.OuterIterationMax = 2;
params.InnerIterationMax = 20;
nmv = 1;

params = initMaskNMPC(params,2,2,1,0,0,Ts,'CGMRES');

% Reference value
ref = [2;0];

%Calculation for initial inputs which is satisfied "dh/du = 0" by Newton method
u0 = rand(3,1);
lamda0 = wrap_dPhidx(x0,ref,params);
while(true)
    u0 = u0 - wrap_ddHddu(x0,lamda0,u0,0,ref,params) \ wrap_dHdu(x0,lamda0,u0,0,ref,params);
    if norm(wrap_dHdu(x0,lamda0,u0,0,ref,params)) < 1e-6
        disp('Newton step was converged!')
        params.initialConditionInputs = u0;
        break;
    end
end

% Simulation loops
Tsim = 10;
N = Tsim/Ts;
Xlog = [];Xlog = [Xlog,x0];
Ulog = [];Ulog = [Ulog,u0];

Tinc = 0;
du0 = zeros(length(params.initialConditionInputs)*params.divideNumber,1);
u0 = repmat(params.initialConditionInputs,params.divideNumber,1);
sys = c2d(ss(tf(1,[0.01 1])),Ts);
%%
for i = 1:N
    fprintf('Iteration No.%d\n',i);
    % State evolution
    x = RK4(@statefcn,x0,u0(1:nmv),0,Ts);
    % NMPC
    [du,u,mv] = ngmresfcn(x,u0,du0,0,ref,Tinc,params,nmv);
    % Update solution
    du0 = du;
    u0 = u;
    x0 = x;
    Tinc = sys.a*Tinc+sys.b;
    Xlog = [Xlog,x];
    Ulog = [Ulog,u(1:3)];
end
%%
t = (0:N)*Ts;
figure(1),clf
subplot(3,1,1)
plot(t,Xlog(1,:))
xlabel('time')
ylabel('x1')

subplot(3,1,2)
plot(t,Xlog(2,:))
xlabel('time')
ylabel('x2')

subplot(3,1,3)
plot(t,Ulog(1,:)),hold on
% plot(t,Ulog(2,:)),hold on
% plot(t,Ulog(3,:)),hold on
xlabel('time')
legend('u')
ylabel('u')
