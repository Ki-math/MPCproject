function net = trainNN(net,u,yhid,diff)
% %Backward of output layer
% y_back = diff;   %Difference between NN and Target output
% 
% %Backward of hidden layer
% h_back = zeros(net.NUM_HIDDEN,1);
% net_input = net.W2(:,1:end-1)'*y_back;
% h_back = net_input.*(1 - yhid.^2);
% 
% %Updating weights
% % Input to Hidden layer
% W1_old = net.W1;
% W1_new = net.W1 - net.epsilon * h_back * [u;1]';
% net.W1 = (1 + net.alpha) * W1_new - net.alpha * W1_old;
% 
% %Hidden to Output layer
% W2_old = net.W2;
% W2_new = net.W2 - net.epsilon * y_back * [yhid;1]';
% net.W2 = (1 + net.alpha) * W2_new - net.alpha * W2_old;

% Normalize params
umax = net.umax;
umin = net.umin;
ymax = net.ymax;
ymin = net.ymin;

%Backward of output layer
y_back = 2./(ymax-ymin).*diff;   %Difference between NN and Target output

%Backward of hidden layer
h_back = zeros(net.NUM_HIDDEN,1);
net_input = net.W2(:,1:end-1)'*y_back;
h_back = net_input.*(1 - yhid.^2);

%Updating weights
% Input to Hidden layer
uin = normalizeNN(umin,umax,u(1:net.inNode,1));
yfbin = normalizeNN(ymin,ymax,u(net.inNode+1:end));
u = [uin;yfbin];
W1_old = net.W1;
W1_new = net.W1 - net.epsilon * h_back * [u;1]';
net.W1 = (1 + net.alpha) * W1_new - net.alpha * W1_old;

%Hidden to Output layer
W2_old = net.W2;
W2_new = net.W2 - net.epsilon * y_back * [yhid;1]';
net.W2 = (1 + net.alpha) * W2_new - net.alpha * W2_old;

end