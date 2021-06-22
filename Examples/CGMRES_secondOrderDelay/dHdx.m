function y = dHdx(x,lamda,u,d,r,params)
%#codegen
y = [(x(1)-r(1))*params.Q(1)-lamda(2);
     (x(2)-r(2))*params.Q(2)+lamda(1)-lamda(2)*2*0.1];
end