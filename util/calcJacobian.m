function out = calcJacobian(x,f)
%#codegen

n = length(x);
evalf = f(x);
m = size(evalf,1);
out = zeros(n,m);
xper = x;
tollerance = 1e-4;

for i = 1:m
    for j = 1:n
        xper(j) = x(j)+tollerance;
        evalf_per = f(xper);
        out(j,i) = (evalf_per(i) - evalf(i))/tollerance;
        xper = x;
    end
end

end