function iseq = isequalWithTol(num1, num2, relTol, zeroTol) 

iseq = 1;
if nargin < 4
    zeroTol = 1e-5;
end

%% both larger than zero and relative difference is large
if (norm(num1) > zeroTol && norm(num2) > zeroTol) && norm(num1 - num2) / norm(num1) > relTol
  % num1, num2, norm(num1 - num2) / norm(num1)
  iseq = 0;
end