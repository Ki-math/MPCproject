function [A,B,Bd,C,U,MD,Y,X,f0] = calcSuccessiveLinear(T,CA,CAi,Ti,Tc,Ts)
%#codegen
% Define constant outputs
Cc = eye(2);
Dc = zeros(2,2);
% Nominal U are obtained from measurements
U = Tc;
% Nominal MD are obtained from measurements
MD = Ti;
% Nominal X and Y are obtained from estimated MPC states
Y = [CA; T];
X = [CA; T];
% Analytical linearization of mechanistic CSTR model (continuous time)
[Ac, Bc, Bdc] = getLinearContinuous(T,CA);
nx = size(Ac,1);
A = eye(nx)+Ts*Ac;  %Euler approximation
B = Ts*Bc;
Bd = Ts*Bdc;
C = Cc;
f0 = Ts * nonlinearCSTR(T,CA,CAi,Ti,Tc);
end
%EOF
