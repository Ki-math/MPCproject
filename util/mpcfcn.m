function [mv,slack,status] = mpcfcn(x0,mvin,md,ref,sys,params,index,index2,index3,index4,index5)
%#codegen
% MPC funtion : Author by Kohei Iwamoto
% Created in 27/12/2019
%
% Arguments
% x0:Initial observed value(OV), Engineering unit
% mvin:Last manipulated value(MV) for predict steps Hc, Engineering unit
% md:Measurement Disturbanceinput(DV) This is constant value for entire future steps, Engineering unit
% ref:Reference value, Enginerring unit
% sys:Linear Plant model structure of (A B C D)
% params:MPC design parameters
% Dimenstion of each matrix:
% index = n*Hp
% index2 = m*Hc
% index3 = l*Hp
% index4 = (m*Hc)+1
% index5 = (4*index2)+(2*index3)+1
%
% Outputs
% mv:Optimized MV, Engineering unit
% slack:Slack variable of soft constrainst it's is nonnegative
% status:Optimization flag (1:feasible 0:infeasible)

% Permanent variable
persistent dmv0;
persistent iA0;
persistent lamda;
persistent s;

% Dimenstions of output,state,input and disturbance
l = size(sys.C,1);
n = size(sys.C,2);
m = size(sys.B,2);
o = size(sys.f0,1);
nd = size(sys.Bd,2);

% Scale Factor fou MV, MD and OV, X is not required scaling.
Uscale = params.MV_scalefactor;
Yscale = params.OV_scalefactor;
Dscale = params.MD_scalefactor;

% Initialization
% Linear Plant model
A = sys.A;                   % engineering unit
B = sys.B*diag(Uscale);      % design value is dimensionless, thus converintg to engineering unit
Bd = sys.Bd*diag(Dscale);    % disturbance is dimensionless, thus converintg to engineering unit
C = diag(1./Yscale)*sys.C;   % x has engineering thus, converintg to dimensionless
%Nominal operating condition for MV, OV, State, State increment
uoff = sys.U;       % engineering unit
mdoff = sys.MD;     % engineering unit
xoff = sys.X;       % engineering unit
yoff = sys.Y;       % engineering unit
f0 = sys.f0;        % engineering unit

% Control parameters
Hp = params.PredictiveHorizon;   %Predictive horizon
Hc = params.ControlHorizon;      %Control horizon
iterMax = params.IterationMax;   %Max iteration numver for QP solver

% Initialization
mv = zeros(index2,1);
slack = 0;
status = false;

% Construct each matrix of the linear predictive model
% Model form (Engineering unit)
% Z(K) = PSI*x + GAMMA*lastmv + THETA*d_MV + GAMMAd*d0 + PHI*f0
PSI = zeros(index,n);
temp = 1:n;
for i = 1:Hp
    PSI(temp,:) = A^i;
    temp = temp + n;
end

GAMMA = zeros(index,m);
temp1 = 1:n;
for i = 1:Hp
    temp2 = zeros(n,m);
    for j = 1:i
        temp2 = temp2 + A^(j-1)*B;
    end
    GAMMA(temp1,:) = temp2;
    temp1 = temp1 + n;
end

THETA = zeros(index,index2);
temp3 = 1:n;
for i = 1:Hp
    temp4 = 1:m;
    for j = 1:Hc
        temp5 = zeros(n,m);
        for k = 0:(i-j)
            temp5 = temp5 + A^k*B;
        end
        THETA(temp3,temp4) = temp5;
        temp4 = temp4 + m;
    end
    temp3 = temp3 + n;
end

PHI = zeros(index,o);
temp6 = 1:n;
for i = 1:Hp
    temp7 = zeros(n);
    for j = 0:i-1
         temp7 = temp7 + A^j;
    end
    PHI(temp6,1:o) = temp7;
    temp6 = temp6 + o;
end

GAMMAd = zeros(index,nd);
temp7 = 1:n;
for i = 1:Hp
    temp8 = zeros(n,nd);
    for j = 1:i
        temp8 = temp8 + A^(j-1)*Bd;
    end
    GAMMAd(temp7,:) = temp8;
    temp7 = temp7 + n;
end

% Target of state value
T = zeros(index3,1);
diff = (ref - yoff)./Yscale;      % dimensionless
T = kron(ones(Hp,1),diff);

% Weigth of output
Q = zeros(index3);
Q = kron(eye(Hp),params.Q);

% Weight of input
R = zeros(index2);
R = kron(eye(Hc),params.R);

% Mesuement matrix
Ctilde = zeros(index3,index);
Ctilde = kron(eye(Hp),C);

% Product by Ctilde (Dimensionless)
% Initialization
PSI_tilde = zeros(index3,n);
GAMMA_tilde = zeros(index3,m);
GAMMAd_tilde = zeros(index3,nd);
THETA_tilde = zeros(index3,index2);
PHI_tilde = zeros(index3,o);
PSI_tilde = Ctilde*PSI;
GAMMA_tilde = Ctilde*GAMMA;
GAMMAd_tilde = Ctilde*GAMMAd;
THETA_tilde = Ctilde*THETA;
PHI_tilde = Ctilde*PHI;

% Remove offset from state
x0 = x0 - xoff;

% Calculate QP params
% Cost Function Form : J = 1/2*sum((y(k)-ref)'*Q*(y(k)-ref)+du(k)'*R*du(k))
Htemp = zeros(index4);
H = zeros(index4);
M = zeros(index4,1);
lastmv = zeros(m,1);
LAMDA = zeros(index,1);
lastmv = (mvin(1:m,1) - uoff)./Uscale;    % Remove offset and convert to dimensionless
md0 = (md - mdoff)./Dscale;               % Remove offset from measurement disturbance and convert to dimensionless 

if ~isempty(md)
    LAMDA = T - (PSI_tilde*x0 + GAMMA_tilde*lastmv + GAMMAd_tilde*md0 + PHI_tilde*f0);  % dimensionless
else
    LAMDA = T - (PSI_tilde*x0 + GAMMA_tilde*lastmv + PHI_tilde*f0);                     % dimensionless
end
Htemp = [THETA_tilde'*Q*THETA_tilde+R zeros(m*Hc,1);     % Hessian
         zeros(1,m*Hc) params.rho];
H = (Htemp+Htemp')/2;                                    % Convert to Symmetric matrix
M = [-THETA_tilde'*Q*LAMDA;0];                           % Linear coefficient of QP

% Calculate constraints
% Constraint of input magnitude
Amv = zeros(index2,index4);
temp_Amv = kron(tril(ones(Hc)),eye(m));
Amv = [temp_Amv zeros(index2,1)];
bmv_max = zeros(index2,1);
bmv_min = zeros(index2,1);
bmv_max = kron(ones(Hc,1),max(zeros(m,1),(params.MV_max-uoff)./Uscale-lastmv));     % Consider nominal point U
bmv_min = kron(ones(Hc,1),max(zeros(m,1),-(params.MV_min-uoff)./Uscale+lastmv));    % Dimensionless

% Constraits of input rate
Admv = zeros(index2,index4);
Admv = [eye(index2) zeros(index2,1)];
bdmv_max = zeros(index2,1);
bdmv_min = zeros(index2,1);
bdmv_max = kron(ones(Hc,1),params.dMV_max./Uscale);     % Dimensionless
bdmv_min = kron(ones(Hc,1),-params.dMV_min./Uscale);

% Constraonts of Output
Vmax = zeros(index3,1);
Vmin = zeros(index3,1);
Vmax = params.softConstRelaxation(1)*ones(index3,1);
Vmin = params.softConstRelaxation(2)*ones(index3,1);
Aov_max = zeros(index3,index4);
Aov_min = zeros(index3,index4);
Aov_max = [THETA_tilde -Vmax];
Aov_min = -[THETA_tilde Vmin];
bov_max = zeros(index3,1);
bov_min = zeros(index3,1);

if ~isempty(md)
    d = PSI_tilde*x0 + GAMMA_tilde*lastmv + GAMMAd_tilde*md0 + PHI_tilde*f0;    % dimensionless
else
    d = PSI_tilde*x0 + GAMMA_tilde*lastmv + PHI_tilde*f0;   % dimensionless
end
bov_max = kron(ones(Hp,1),(params.OV_max - yoff)./Yscale) - d;
bov_min = kron(ones(Hp,1),-(params.OV_min - yoff)./Yscale) + d;

% Constraints of slack
Aslk = zeros(1,index4);
Aslk = -[zeros(1,index2) 1];
bslk = 0;

% Combined all constraints
Ainq = zeros(index5,index4);
binq = zeros(index5,1);
Ainq = [Amv;-Amv;Admv;-Admv;Aov_max;-Aov_min;Aslk];
binq = [bmv_max;bmv_min;bdmv_max;bdmv_min;bov_max;bov_min;bslk];

% Initialization of permanent var
if isempty(dmv0)
    dmv0 = zeros(index4,1); % +1 is for slack variable
end

% Solve QP problem by optimization solver
if params.SolverType == 1
    % Solve QP problem by Active Set Note:Ainq*x =< binq
    if isempty(iA0)
        iA0 = zeros(index5,2);
        for i = 1:index5
           diff = Ainq(i,index4)*dmv0 - binq(i);
           if abs(diff) <= 1e-6
               iA0(i,1) = 1;   %ActiveSet Flag true:1
           end
           iA0(i,2) = 1;       %Inequality constarint id:1
        end
    end
    [dmv0,~,~,iA0,status] = activeSet(dmv0,H,M,Ainq,binq,[],[],iA0,iterMax);
else
    % Solve QP problem by Interior Point Method Note:Ainq*x >= binq 
    if isempty(lamda) && isempty(s)
        lamda = ones(index5,1); 
        s = ones(index5,1);
    end
    [dmv0,~,status] = interiorPointMethod_inq(dmv0,lamda,s,H,M,-Ainq,-binq,iterMax);
   
    % Build in Solver by MPC Toolbox
%     exitflag = 0;
%     opt = mpcInteriorPointOptions;
%     [dmv0,exitflag] = mpcInteriorPointSolver(H,M,Ainq,binq,zeros(0,index4),zeros(0,1),dmv0,opt);
%     if exitflag > 0
%         status = true;
%     end
    
    if ~status
        % If the solver cannot converge to a solution, some variables are reset.
        lamda = ones(index5,1); 
        s = ones(index5,1);
    end
end

% Update MV
if status % true = feasible
    scaleMatrix = kron(eye(Hc),diag(Uscale));
    mv(1:index2,1) = tril(kron(ones(Hc),eye(m)))*scaleMatrix*dmv0(1:index4-1)+kron(ones(Hc,1),eye(m))*(Uscale.*lastmv + uoff);
    slack = dmv0(end);   %Last element is slack
else
    mv(1:index2,1) = mvin;
    slack = 0;
end

end
% EOF