function [a, b, bd] = getLinearContinuous(T,CA)
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
drAdCA = k;
drAdT = CA*k*EoR/(T^2);

% Continuous-time matrix: A
a = [(drAdCA - FoV) drAdT;
     dHrCon*drAdCA (dHrCon*drAdT - FoV - UACon)];
% Continuous-time matrix: B
b = [0;UACon];
% Continuous-time matrix: Bd
bd = [0;FoV];
end
