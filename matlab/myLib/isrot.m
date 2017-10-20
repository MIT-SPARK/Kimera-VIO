function x = isrot (a, tol)
% Use: bool = isrot(R) (counterintuitive: return 0 if it's a rotation)
% Checks if the 3x3 matrix R is a rotation matrix
%      isrot(R)==0 rotation matrix
%      isrot(R)~=0 not a rotation
%

if nargin < 2
    tol = 1e-5;
end

[r c] = size(a);
x = 1;
if r ~= 3
    disp ('Matrix has not 3 rows')
elseif c ~= 3
    disp ('Matrix has not 3 columns')
elseif norm ( a * a' - eye(3) ) > tol
    disp ('Matrix is not orthonormal, i.e. ||(R''R-I)|| > 1E-10')
    disp(norm ( a * a' - eye(3) ))
    disp('-----------------------')
elseif det (a) < 0
    disp ('Matrix determinant is -1')
else x = 0;
end