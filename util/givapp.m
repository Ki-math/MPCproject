function vrot=givapp(c,s,vin,k)
% #codegen
%  Apply a sequence of k Givens rotations, used within gmres codes
%  C. T. Kelley, July 10, 1994 (modified by S. Tajeddin for use in MPsee toolbox)
%  function vrot=givapp(c, s, vin, k)
vrot = vin;
for i = 1:k
    w1 = c(i)*vrot(i)-s(i)*vrot(i+1);
    w2 = s(i)*vrot(i)+conj(c(i))*vrot(i+1);
    vrot(i:i+1) = [w1,w2];
end