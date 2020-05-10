function sys = calcCTSyetem(net,x,u)

persistent nnIn;

if isempty(nnIn)
    nnIn = zeros(net.NUM_INPUT,1);
end

Ts = 0.01;

n = size(x,1);
m = size(u,2);
A = zeros(n);
B = zeros(n,m);
C = eye(n);

A = calcdfdx(net,u,x);
B = calcdfdu(net,u,x);

nnIn(1:net.inOrder) = u;
[f0,~] = predictNN(net,nnIn);

nnIn = delayInput(nnIn,net);
nnIn(net.inNode+1:net.inNode+net.outOrder) = x;        %kÇÃèoóÕ

sys.A = A;
sys.B = B;
sys.C = C;
sys.U = u;
sys.Y = C*x;
sys.X = x;
sys.f0 = f0-x;

end