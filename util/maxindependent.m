function [Aout,bout] = maxindependent(A,b)
%#codegen
%MAXINDEPENDENT takes a matrix A and produces an array in which the columns 
%are a subset of independent vectors with maximum size. 
[r,c]= size(A);
n = rank(A);
% Aout = zeros(r,n);
% bout = zeros(1,n);
% idx = 1;
% for i = 1:c 
%   Aout(:,idx) = A(:,i);
%   bout(:,idx) = b(:,i);
%   if rank(Aout(:,1:idx)) == n
%       break;
%   elseif rank(Aout(:,1:idx)) ~= idx
%       Aout(:,idx) = zeros(r,1);
%       bout(:,idx) = 0;
%       idx = idx - 1;
%   end
%   idx = idx + 1;
% end
Aout = zeros(n,c);
bout = zeros(n,1);
idx = 1;
for i = 1:r 
  Aout(idx,:) = A(i,:);
  bout(idx,1) = b(i,1);
  if rank(Aout(1:idx,:)) == n
      break;
  elseif rank(Aout(1:idx,:)) ~= idx
      Aout(idx,:) = zeros(1,c);
      bout(idx,1) = 0;
      idx = idx - 1;
  end
  idx = idx + 1;
end
end