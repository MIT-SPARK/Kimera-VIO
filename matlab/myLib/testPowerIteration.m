clear all
close all
clc

n = 10;
A = rand(n,n);
A = A'*A;

[vectorsExpected,lambdasExpected] = eig(A);
[lambdaMaxExpected,ind] = max(diag(lambdasExpected));
vectorMaxExpected = vectorsExpected(:,ind);
% check
if norm(A * vectorMaxExpected - lambdaMaxExpected * vectorMaxExpected ) > 1e-5
   error('wrong expected') 
end

% check power iteration
x = rand(n,1);
for i=1:1000
   x = A * x;
   normx = norm(x);
   x = x / normx;
end
% check
if norm(x - vectorMaxExpected) > 1e-5
   error('wrong power iter 1') 
end
if norm(normx - lambdaMaxExpected) > 1e-5
   error('wrong power iter 2') 
end

%%%% CHECK SMALLEST EIG:
[lambdaMinExpected,ind] = min(diag(lambdasExpected));
vectorMinExpected = vectorsExpected(:,ind);
% check
if norm(A * vectorMinExpected - lambdaMinExpected * vectorMinExpected ) > 1e-5
   error('wrong expected min') 
end

lambdaMaxExpected
offset = 1000;
B = -(A - offset * speye(n));
% check power iteration
x = rand(n,1);
for i=1:100000
   x = B * x;
   normx = norm(x);
   x = x / normx;
end
normx = -normx + offset
% check
% if norm(x - vectorMinExpected) > 1e-5
%    error('wrong power iter 1') 
% end
lambdaMinExpected
if norm(normx - lambdaMinExpected) > 1e-5
   error('wrong power iter 2') 
end