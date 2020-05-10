function x = stateFcn(x,u,Ts)
[~,~,~,~,~,~,~,~,f] = mypendulumCT(x,u);
x = x + Ts*f;
end