clc
clear
clear wrapCGMRES

%Simulation parameter
Tsim = 10;
mode = 'SL';

%Parameters for MPC
nmv = 1;
Ts = 0.001;
params.PredictiveTime = 1;                   % Finite Predictive time[s]
params.divideNumber = 10;                    % Separation number
params.Q = [50;2];                           % Stage Cost Weights for state value
params.Qf = 10*[50;2];                       % Weights for terminal condition
params.R = 1;                                % Weights for input
params.MV_max = 5;                           % Inputs Constraints
params.MV_min = -5;
params.zeta = 10;                            % Decay coefficient
params.stateFunction = 'model';
params.jacobianTerminalFunction = 'dPhidx';
params.jacobianHamiltonianState = 'dHdx';
params.jacobianHamiltonianInput = 'dHdu';

%Reference signal
ref0=[0;0];                                     % Initial
ref=[2;0];                                      % Steady

%State and Input initial values
x0 = [0;0];                                     % States
u0 = [0.1;0.2;0.3];                             % Inputs(includes lagrange and slack var)

lamda0 = dPhidx(x0,ref0,params);                %Initial lagrange variables(costate var)

%Calculation for initial inputs which is satisfied "dh/du = 0" by Newton
%method
while(true)
    
    u0 = u0 - ddHddu(x0,lamda0,u0,params) \ dHdu(x0,lamda0,u0,0,ref,params);
    
    if norm(dHdu(x0,lamda0,u0,0,ref,params)) < 10^-6
        params.u0 = u0;
        break;
    end
    
end
mv0 = u0(1);
params.initialConditionInputs = u0;

%% Simulation
if strcmp(mode,'ML')
    %Run on the ML
    N = Tsim/Ts;

    X = []; X(:,1) = x0;
    U = []; U(:,1) = mv0;

    for i = 1:N
       fprintf(1,'Iteration No.%d\n',i)
       x = RK4(@model,X(:,i),U(:,i),0,Ts);
       tic
       [u,status] = wrapCGMRES(U(:,i),x,ref,0,params,nmv);
       toc
       X = [X,x];
       U = [U,u];
    end

    t = (0:N)'*Ts;

    figure(1),clf
    subplot(2,1,1)
    plot(t,X(1,:)),hold on
    plot(t,X(2,:)),hold off
    legend('pos','velo')
    ylabel('output')

    subplot(2,1,2)
    plot(t,U(1,:))
    xlabel('time[s]')
    ylabel('input')
   
else
    %Run on the SL
    open_system('testModel')
    tic
    sim('testModel',Tsim)
    toc
end
