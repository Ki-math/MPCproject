function params = initMaskNMPC(params,nx,ny,nu,ns,nL,Ts,kindMPC)
% Set default params
if strcmp(kindMPC,'SQP')
    % SQP
    if ~isfield(params, 'Q')
        params.Q = diag(ones(1,ny));
    end
    if ~isfield(params, 'R')
        params.R = diag(ones(1,nu));
    end
    if ~isfield(params, 'PredictiveHorizon')
        params.PredictiveHorizon = 10;
    end
    if ~isfield(params, 'ControlHorizon')
        params.ControlHorizon = 5;
    end
    if ~isfield(params, 'MV_max')
        params.MV_max = 999*ones(nu,1);
    end
    if ~isfield(params, 'MV_min')
        params.MV_min = -999*ones(nu,1);
    end
    if ~isfield(params, 'dMV_max')
        params.dMV_max = 999*ones(nu,1);
    end
    if ~isfield(params, 'dMV_min')
        params.dMV_min = -999*ones(nu,1);
    end
    if ~isfield(params, 'OV_max')
        params.OV_max = 999*ones(ny,1);
    end
    if ~isfield(params, 'OV_min')
        params.OV_min = -999*ones(ny,1);
    end
    if ~isfield(params, 'rho')
        params.rho = 10^4;
    end
    if ~isfield(params, 'softConstRelaxation')
        params.softConstRelaxation = ones(2*ny,1);
    end
    if ~isfield(params, 'MV_scalefactor')
        params.MV_scalefactor = ones(nu,1);
    end
    if ~isfield(params, 'OV_scalefactor')
        params.OV_scalefactor = ones(ny,1);
    end
    if ~isfield(params, 'usefmincon')
        params.usefmincon = true;
    end
    if ~isfield(params, 'initialconditionMV')
        params.initialconditionMV = zeros(nu,1);
    end
    if ~isfield(params, 'initialconditionState')
        params.initialconditionState = zeros(nx,1);
    end
    if ~isfield(params, 'IterationMax')
        params.IterationMax = 50;
    end
%     if ~isfield(params, 'stateFunction')
%         error('Error:Please set the State function');
%     end
%     if ~isfield(params, 'measurementFunction')
%         error('Error:Please set the Measurement function');
%     end
elseif strcmp(kindMPC,'CGMRES')
    % CGMRES
    if ~isfield(params, 'Q')
        params.Q = ones(nx,1);
    end
    if ~isfield(params, 'Qf')
        params.Qf = ones(nx,1);
    end
    if ~isfield(params, 'R')
        params.R = ones(nu,1);
    end
    if ~isfield(params, 'PredictiveTime')
        params.PredictiveTime = 1;
    end
    if ~isfield(params, 'divideNumber')
        params.divideNumber = 10;
    end
    if ~isfield(params, 'raiseGain')
        params.raiseGain = 1;
    end
    if ~isfield(params, 'MV_max')
        params.MV_max = 999*ones(nu,1);
    end
    if ~isfield(params, 'MV_min')
        params.MV_min = -999*ones(nu,1);
    end
    if ~isfield(params, 'zeta')
        params.zeta = 1/Ts;
    end
    if ~isfield(params, 'h')
        params.h = 0.001;
    end
    if ~isfield(params, 'initialConditionInputs')
        params.initialConditionInputs = zeros(nu+ns+nL,1);
    end
    if ~isfield(params, 'IterationMax')
        params.IterationMax = nu*params.divideNumber;
    end
%     if ~isfield(params, 'stateFunction')
%         error('Error:Please set the State function');
%     end
%     if ~isfield(params, 'jacobianTerminalFunction')
%         error('Error:Please set the Jacobian about Terminal function');
%     end
%     if ~isfield(params, 'jacobianHamiltonianState')
%         error('Error:Please set the Jacobian of state about Hamiltonian');
%     end
%     if ~isfield(params, 'jacobianHamiltonianInput')
%         error('Error:Please set the Jacobian of Input about Hamiltonian');
%     end
    
    temp = c2d(ss(tf(params.PredictiveTime,[(1/params.raiseGain),1])),Ts);
    params.T_a = temp.a;                         % Discrete system of the predictive time
    params.T_b = temp.b;                         % Discrete system of the predictive time
else
    % Newton-GMRES
    if ~isfield(params, 'Q')
        params.Q = ones(nx,1);
    end
    if ~isfield(params, 'Qf')
        params.Qf = ones(nx,1);
    end
    if ~isfield(params, 'R')
        params.R = ones(nu,1);
    end
    if ~isfield(params, 'PredictiveTime')
        params.PredictiveTime = 1;
    end
    if ~isfield(params, 'divideNumber')
        params.divideNumber = 10;
    end
    if ~isfield(params, 'raiseGain')
        params.raiseGain = 1;
    end
    if ~isfield(params, 'MV_max')
        params.MV_max = 999*ones(nu,1);
    end
    if ~isfield(params, 'MV_min')
        params.MV_min = -999*ones(nu,1);
    end
    if ~isfield(params, 'h')
        params.h = 0.001;
    end
    if ~isfield(params, 'initialConditionInputs')
        params.initialConditionInputs = zeros(nu+ns+nL,1);
    end
    if ~isfield(params, 'InnerIterationMax')
        params.InnerIterationMax = nu*params.divideNumber;
    end
    if ~isfield(params, 'OuterIterationMax')
        params.OuterIterationMax = 5;
    end
%     if ~isfield(params, 'stateFunction')
%         error('Error:Please set the State function');
%     end
%     if ~isfield(params, 'jacobianTerminalFunction')
%         error('Error:Please set the Jacobian about Terminal function');
%     end
%     if ~isfield(params, 'jacobianHamiltonianState')
%         error('Error:Please set the Jacobian of state about Hamiltonian');
%     end
%     if ~isfield(params, 'jacobianHamiltonianInput')
%         error('Error:Please set the Jacobian of Input about Hamiltonian');
%     end
    
    temp = c2d(ss(tf(params.PredictiveTime,[(1/params.raiseGain),1])),Ts);
    params.T_a = temp.a;                         % Discrete system of the predictive time
    params.T_b = temp.b;                         % Discrete system of the predictive time    
end

end