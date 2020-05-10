function nnInout = delayInput(nnIn,net)
nnInout = zeros(length(nnIn),1);
num_in = net.inOrder;
num_out = net.outOrder;
indelay = net.indelay;
outdelay = net.outdelay;


%Delay In/Output
if indelay ~= 0
    nnInout(num_in+1:(1+indelay)*num_in) = nnIn(1:indelay*num_in);
end

if outdelay ~= 0
    nnInout((1+indelay)*num_in+num_out+1:end) = nnIn((1+indelay)*num_in+1:end-num_out);
end

end