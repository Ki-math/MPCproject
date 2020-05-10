function xd = f(x,u,Ts)
% Calculate the discrete nonlinear dynamics f(x(k),u(k))

% State x = [CA,T]' Input = [Tc,Ti]'
CA = x(1);
T = x(2);
Tc = u(1);
Ti = u(2);
CAi = 10;       % Constant value

% Nonlinear model time derivatives
f = zeros(length(x),1);
f = nonlinearCSTR(T,CA,CAi,Ti,Tc);

% Convert to discrete system by the difference method
xd = x + Ts*f;

end
%EOF
