function [params,sysm] = initMaskMPC(params,sysm,numState,numRef,numMV,numDist,kindMPC)

l = numRef;
m = numMV;
nd = numDist;

% Set default sysm
if strcmp(kindMPC,'MPC')
    if ~isfield(sysm, 'Bd')
        sysm.Bd = zeros(numState,numDist);
    end
    if ~isfield(sysm, 'U')
        sysm.U = zeros(numMV,1);
    end
    if ~isfield(sysm, 'MD')
        sysm.MD = zeros(numDist,1);
    end
    if ~isfield(sysm, 'Y')
        sysm.Y = zeros(numRef,1);
    end
    if ~isfield(sysm, 'X')
        sysm.X = zeros(numState,1);
    end
    if ~isfield(sysm, 'f0')
        sysm.f0 = zeros(numState,1);
    end
end

% Set default params
if ~isfield(params, 'Q')
    params.Q = diag(ones(1,l));
end
if ~isfield(params, 'R')
    params.Q = diag(ones(1,m));
end
if ~isfield(params, 'PredictiveHorizon')
    params.PredictiveHorizon = 10;
end
if ~isfield(params, 'ControlHorizon')
    params.ControlHorizon = 5;
end
if ~isfield(params, 'MV_max')
    params.MV_max = 999*ones(m,1);
end
if ~isfield(params, 'MV_min')
    params.MV_min = -999*ones(m,1);
end
if ~isfield(params, 'dMV_max')
    params.dMV_max = 999*ones(m,1);
end
if ~isfield(params, 'dMV_min')
    params.dMV_min = -999*ones(m,1);
end
if ~isfield(params, 'OV_max')
    params.OV_max = 999*ones(l,1);
end
if ~isfield(params, 'OV_min')
    params.OV_min = -999*ones(l,1);
end
if ~isfield(params, 'rho')
    params.rho = 10^4;
end
if ~isfield(params, 'softConstRelaxation')
    params.softConstRelaxation = ones(2*l,1);
end
if ~isfield(params, 'MV_scalefactor')
    params.MV_scalefactor = ones(m,1);
end
if ~isfield(params, 'OV_scalefactor')
    params.OV_scalefactor = ones(l,1);
end
if ~isfield(params, 'MD_scalefactor')
    params.MD_scalefactor = ones(nd,1);
end
if ~isfield(params, 'SolverType')
    params.SolverType = 1;  %Active set
end
if ~isfield(params, 'initialconditionMV')
    params.initialconditionMV = zeros(m,1);
end
if ~isfield(params, 'IterationMax')
    params.IterationMax = 50;
end

end