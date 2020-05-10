function Pl = calcPl(x,dx,J,g,h,rho)
%#codegen
%Exact penalty function
costTerm = J(x);
inequalityTerm = 0;
f = calcJacobian(x,J);
gd = calcJacobian(x,g);
hd = calcJacobian(x,h);
if ~isempty(g)
   inequalityTerm = sum(max(0,g(x)+gd'*dx));
end
equalityTerm = 0;
if ~isempty(h)
   equalityTerm = sum(abs(h(x)+hd'*dx));
end

Pl = costTerm + f'*dx + rho*(inequalityTerm + equalityTerm);
end