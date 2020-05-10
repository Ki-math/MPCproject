function [y,yhid] = predictNN(net,u)
%Predict
umax = net.umax;
umin = net.umin;
ymax = net.ymax;
ymin = net.ymin;

uin = normalizeNN(umin,umax,u(1:net.inNode,1));
yfbin = normalizeNN(ymin,ymax,u(net.inNode+1:end));
u = [uin;yfbin];

yhid = activationFcn_hid(net.W1*[u;1]);
ynn = net.W2*[yhid;1];
y = reverseNN(ymin,ymax,ynn);
end

%Activation function of hidden layer
function y = activationFcn_hid(x)
y = tanh(x);
end
%EOF