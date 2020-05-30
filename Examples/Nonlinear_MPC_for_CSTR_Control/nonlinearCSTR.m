function dxdt = nonlinearCSTR(T,CA,CAi,Ti,Tc)
% Define linear continuous-time matrices A and B for CSTR.  Last input is
% offset.

% Parameters
EaOverR = 5963.6;
lnA0 = 17.3689;
FoV = 1;                    % F/V [=] 1/s
A0 = exp(lnA0);             % Arrhenius pre-exponential, [=] 1/s
EoR = EaOverR;              % Ea/R [=] K
dHrCon = -11.92;            % delH/(Rho*C) [=] K-m3/kmol
UACon = 0.3;                % UA/(Rho*C*V) [=] 1/s
k = -A0*exp(-EoR/T);        % k is for rate of A production [=] 1/s
rA = k*CA;

% Nonlinear model time derivatives
dxdt = zeros(2,1);
dxdt(1) = (CAi - CA)*FoV + rA;
dxdt(2) = (Ti - T)*FoV + UACon*(Tc -T) + dHrCon*rA;
%EOF
