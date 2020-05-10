%Normailize Input
function y = normalizeNN(min,max,u)
n = size(u,1)/length(min);
MIN = kron(ones(n,1),min);
MAX = kron(ones(n,1),max);
y = 2.*(u-MIN)./(MAX-MIN) - 1;
end