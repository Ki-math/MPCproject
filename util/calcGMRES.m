function x = calcGMRES(A,b,x)
%A:Left hand side Matrix
%b:Right hand side vector
%x:Initial guess state

%Reference: http://www.cit.nihon-u.ac.jp/kouendata/No.43/7_sujo/7-036.pdf

%For tollerance
tol = 10^-6;

%Max iterlation
max_iter = 10;

%2 norm of b for normalization
bnrm2 = norm(b);
if bnrm2 <= tol
    bnrm2 = 1;
end
    
%Initial residual
r0 = b - A * x;
error = norm(r0)/bnrm2;
if error <= tol
    %If error is small, it's end
    return;
end

%State Order GMRES is guaranteed to be abel to converge under this order.  @
n = size(A,1);

%Restart parameter
%To avoid memory leak due to large matrix, iter is reset by this parameter.
m = 30;                  

%Matrix v has orthonormal
v = zeros(n,m+1);       %row:n column:m+1

%Hessenberg matrix
h = zeros(m+1,m);       %row:m+1 column:m

c = zeros(m,1);
s = zeros(m,1);
e1 = zeros(n,1);
e1(1) = 1;

for k = 1:max_iter
    
    %residual error
    r = b - A * x;
    v(:,1) = r / norm(r);
    g = norm(r)*e1;
    
    for j = 1:m
        v_hat = A * v(:,j);
       
        for i = 1:j
            h(i,j) = v_hat' * v(:,i);
            v_hat = v_hat - h(i,j) * v(:,i);
        end

        h(j+1,j) = norm(v_hat);
        v(:,j+1) = v_hat/h(j+1,j);

        %Givens rotation
        for i = 1:j-1
            temp = c(i)*h(i,j) + s(i)*h(i+1,j);
            h(i+1,j) = -s(i)*h(i,j) + c(i)*h(i+1,j);
            h(i,j) = temp;
        end

        [c(j),s(j)] = rotmat( h(j,j), h(j+1,j) );   % form j-th rotation matrix
        temp   = c(j)*g(j);                         % approximate residual norm
        g(j+1) = -s(j)*g(j);
        g(j)   = temp;
        h(j,j) = c(j)*h(j,j) + s(j)*h(j+1,j);
        h(j+1,j) = 0.0;
        error  = abs(g(j+1)) / bnrm2;

        if (error <= tol)
            y = h(1:j,1:j) \ g(1:j);
            x = x + v(:,1:j)*y;
            break;
        end
    
    end
    
    if error <= tol
        break;
    end
    
    %If iter dosen't converge under n-th order, re-start by updating...
    %solution
    y = h(1:m,1:m) \ g(1:m);
    x = x + v(:,1:m)*y;
    r = b - A * x;
    error = norm(r)/bnrm2;
    if error <= tol
        break;
    end
    
end

if error > tol
    %If solution doesn't find, generating error
    disp('Can not find solution');
end

end

function [ c, s ] = rotmat( a, b )

%
% Compute the Givens rotation matrix parameters for a and b.
%
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