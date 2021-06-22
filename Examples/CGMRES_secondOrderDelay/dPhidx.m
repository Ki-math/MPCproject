function y = dPhidx(x,r,params)
    %I’[ğŒ‚Ì•Î”÷•ª
    y = [(x(1)-r(1)) * params.Qf(1);
         (x(2)-r(2)) * params.Qf(2)];
end