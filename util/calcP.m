function P = calcP(x,J,g,h,rho)
%#codegen
%Exact penalty function
costTerm = J(x);
inequalityTerm = 0;
if ~isempty(g)
   inequalityTerm = sum(max(0,g(x)));
end
equalityTerm = 0;
if ~isempty(h)
   equalityTerm = sum(abs(h(x)));
end
    
P = costTerm + rho*(inequalityTerm + equalityTerm);
end