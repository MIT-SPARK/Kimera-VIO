function q = dc2quat(dc)
% dc2quat quaternion direction cosine matrix angle axis
%**********************************************************************
%
% dc2quat calculates the quaternion corresponding to a direction
% cosine matrix.
% Assumes input dc is orthonormal.
%
% Input: dc = 3x3 direction cosine matrix
%
% Output: q = quaternion, q(1) = scalar, q(2:4) = vector
% Rotation sense = Successive rotations are right multiplies.
%
% Programmer: James Tursa
%
%**********************************************************************

q = [0 0 0 0];
tr = dc(1,1) + dc(2,2) + dc(3,3);
ii = 0;
nn = 0;
q(1) = tr;
for kk=1:3
    if( dc(kk,kk) > q(1) )
        ii = kk;
        q(1) = dc(kk,kk);
    end
end

tr = sqrt(1 + 2*q(1) - tr);

order = [2 3 1];
for mm=1:3
    kk = order(mm);
    nn = nn + 1;
    jj = 6 - nn - kk;
    x = ii * (nn - ii);
    if( x == 0)
        q(1) = (dc(jj,kk) - dc(kk,jj)) / tr;
        q(nn+1) = q(1);
    else
        q(jj+kk-ii+1) = (dc(jj,kk) + dc(kk,jj)) / tr;
    end
end

if( ii == 0 )
    q(1) = tr;
else
    q(ii+1) = tr;
end
q(2:4) = -q(2:4);
if( q(1) == 0 )
    q = 0.5 * q;
else
    q = 0.5 * sign(q(1)) * q;
end

%\
% MAKES QUATERNION A POSITIVE ROTATION
%/

if( q(1) <= 0 )
    q = -q;
end

%\
% NORMALIZE QUATERNION (KEEPS ROUTINE STABLE)
%/

q = q / norm(q);

return

end