function [duopt,status] = cgmresfcn(x0,u0,du0,d,ref,T,params)
%#codegen
% C/GMRES funtion : Author by Kohei Iwamoto
% Created in 24/5/2020
% 
% Arguments
% x0:Intial State [n,1]
% u0:Previous Input [nu*divideTime,1]
% du0:Previous rate of input [nu*divideTime,1]
% d:Disturbance [nd,1]
% ref:Refernce value of state [ny,1]
% T:Prediction time Scalar
% params:Parameter Structure
% 
% Outputs
% duopt:Rate of input [nu*divideTime,1]
% status:Optimization Flag True or False Boolean

% Initialization
duopt = zeros(length(du0),1,'like',du0);
du = zeros(length(du0),1,'like',du0);
du = du0;
status = false;
tol = 10^-6;                                        % Tollerance value
step = 1e-3;                                        % Small constant value
max_iter = params.IterationMax;
ts = T/params.divideNumber;                         % Sampling time during current prediction time
model = str2func(params.stateFunction);             % Function handle for the state function
dPhidx = str2func(params.jacobianTerminalFunction); % Function handle for the Jacobian of the terminal function phi
dHdx = str2func(params.jacobianHamiltonianState);   % Function handle for the Jacobian of the hamiltonian function H about state
dHdu = str2func(params.jacobianHamiltonianInput);   % Function handle for the Jacobian of the hamiltonian function H about input
n = length(u0);                                     % GMRES is guaranteed to be able to converge under the order n.Å@

% Differential of state
dx = model(x0,u0,d);

% Nonlinear Algebraic Equation
F = calcDiff(x0,u0,d,ref,ts,params,model,dPhidx,dHdx,dHdu);                                    
Fx_t = calcDiff(x0+dx*step,u0,d,ref,ts,params,model,dPhidx,dHdx,dHdu); 
Fx_u_t = calcDiff(x0+dx*step,u0+du*step,d,ref,ts,params,model,dPhidx,dHdx,dHdu);

% CGMRES is solved the differential equations by the forward difference approximation 
% dF(x,u,t)/dt = -zeta*F(x,u,t)
% ~=(F(x+h*xdot,u+h*udot,t+h) - F(x,u,t))/h = -zeta*F(x,u,t)
% ~=(F(x+h*xdot,u+h*udot,t+h) - F(x+h*xdot,u,t+h))/h*udot = -zeta*F(x,u,t) - (F(x+h*xdot,u,t+h))-F(x,u,t))/h 
Right = -params.zeta * F - ((Fx_t - F)/step);  %b
Left = (Fx_u_t - Fx_t)/step;                   %A*x

% Calc L2 norm of the b for normalization
bnrm2 = norm(Right);
if bnrm2 <= tol
    bnrm2 = 1;
end
    
%Initial residual
r = Right - Left;
error = norm(r)/bnrm2;
if error <= tol
    % If error is small, loop is end
    duopt = du;
    status = true;
    return;
end

% Restart parameter
% To avoid memory leak due to large matrix, iteration is reset by this parameter.
m = 30;                  

% Matrix v is orthonormal
v = zeros(n,m+1);       % row:n column:m+1

% Hessenberg matrix
h = zeros(m+1,m);       % row:m+1 column:m

% Initialization
c = zeros(m,1);
s = zeros(m,1);
g = zeros(n+1,1);
e1 = zeros(n+1,1);
e1(1) = 1;

% GMRES Loop
for k = 1:max_iter
    
    v(1:n,1) = r/norm(r);
    g = norm(r)*e1;
    
    for j = 1:m
        Fx_u_t = calcDiff(x0+dx*step,u0+v(1:n,j)*step,d,ref,ts,params,model,dPhidx,dHdx,dHdu);
        v_hat = (Fx_u_t - Fx_t)/step;            % Difference approximation for A*v
       
        for i = 1:j
            h(i,j) = v_hat' * v(1:n,i);
            v_hat = v_hat - h(i,j)*v(1:n,i);
        end

        h(j+1,j) = norm(v_hat);
        v(1:n,j+1) = v_hat/h(j+1,j);

        % Givens rotation
        for i = 1:j-1
            temp = c(i)*h(i,j) + s(i)*h(i+1,j);
            h(i+1,j) = -s(i)*h(i,j) + c(i)*h(i+1,j);
            h(i,j) = temp;
        end

        [c(j),s(j)] = rotmat(h(j,j),h(j+1,j));   % Form j-th rotation matrix
        temp = c(j)*g(j);                        % Approximate residual norm
        g(j+1) = -s(j)*g(j);
        g(j) = temp;                      
        h(j,j) = c(j)*h(j,j) + s(j)*h(j+1,j);
        h(j+1,j) = 0.0;
        error = abs(g(j+1)) / bnrm2;

        if (error <= tol)
            y = h(1:j,1:j) \ g(1:j);
            duopt = du + v(1:n,1:j)*y;
            status = true;
            return;
        end
    end
    
    % If iter dosen't converge under m times, re-start by updating current solution
    y = h(1:m,1:m)\g(1:m);
    du = du + v(1:n,1:m)*y;
    Fx_u_t = calcDiff(x0+dx*step,u0+du*step,d,ref,ts,params,model,dPhidx,dHdx,dHdu);
    Left = (Fx_u_t - Fx_t)/step;
    r = Right - Left;
    error = norm(r)/bnrm2;
    if error <= tol
        duopt = du;
        status = true;
        break;
    end
end

end

% Compute the Givens rotation matrix parameters for a and b.
function [c,s] = rotmat(a,b)

   if ( b == 0.0 )
      c = 1.0;
      s = 0.0;
   elseif ( abs(b) > abs(a) )
      temp = a / b;
      s = 1.0 / sqrt( 1.0 + temp^2 );
      c = temp * s;
   else
      temp = b / a;
      c = 1.0 / sqrt( 1.0 + temp^2 );
      s = temp * c;
   end
   
end
%EOF