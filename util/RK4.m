function x = RK4(varargin)
%#codegen
%Runge-kutta solver

func = varargin{1}; % Function handle
x0 = varargin{2};   % Current state
u = varargin{3};    % Current Input 
d = varargin{4};    % Disturbance
h = varargin{5};    % Sampling time

k1 = func(x0,u,d);
k2 = func(x0+h/2*k1,u,d);
k3 = func(x0+h/2*k2,u,d);
k4 = func(x0+h*k3,u,d);

x = x0 + h/6 * (k1 + 2*k2 + 2*k3 + k4);
end