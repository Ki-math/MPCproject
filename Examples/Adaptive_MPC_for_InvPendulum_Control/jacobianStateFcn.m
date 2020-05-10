function Ad = jacobianStateFcn(x,u,Ts)
Ac = mypendulumCT(x,u);
Ad = eye(length(x))+Ts*Ac; %Convert to discrete time basen on the Euler approximation
end