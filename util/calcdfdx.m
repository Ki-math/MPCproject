function [A,yhid] = calcdfdx(net,u,x)
%Caluculate A matrix
ymax = net.ymax;
ymin = net.ymin;
umax = net.umax;
umin = net.umin;
un = normalizeNN(umin,umax,u);
yn = normalizeNN(ymin,ymax,x);

n = length(x);
A = zeros(n);

W1 = net.W1;
W2 = net.W2;
inOrder = net.inOrder;
numHidden = net.NUM_HIDDEN;
numOutput = net.NUM_OUTPUT;
z = zeros(numHidden,1);
z = W1*[un;yn;1];
yhid = zeros(numHidden,1);
yhid = tanh(z);
diagterm = diag((ones(numHidden,1)-yhid.^2));

for i = 1:numOutput
  for j = 1:n
      A(i,j) = (ymax(i)-ymin(i)/2)*W2(i,1:end-1)*diagterm*W1(:,inOrder+j)*2/(ymax(j)-ymin(j));
  end
end

end
%EOF