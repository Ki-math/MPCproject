function params = initMaskNMPC(params,nx,ny,nu,nd)

% Set default params
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
if ~isfield(params, 'MD_scalefactor')
    params.MD_scalefactor = ones(nd,1);
end
if ~isfield(params, 'State_scalefactor')
    params.State_scalefactor = ones(nx,1);
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
if ~isfield(params, 'stateFunction')
    error('Error:Please set the State function');
end
if ~isfield(params, 'measurementFunction')
    error('Error:Please set the Measurement function');
end

end