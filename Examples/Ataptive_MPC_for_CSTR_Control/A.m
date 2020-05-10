function Ad = A(x,u,Ts)
% Define the jacobian for the nonlinear dynamics f about state and input
n = length(x);
A = zeros(n); Ad = zeros(n);
CA = x(1); T = x(2);
A = getLinearContinuous(T,CA);

% Sampling time
Ts = 0.1;

% Convert to the discrete system
Ad = eye(n) + Ts*A;

end