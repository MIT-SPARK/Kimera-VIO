clear all
close all
clc

n = 10;
A1 = rand(n,n);
A = A1'*A1;

[vectorsExpected,lambdasExpected] = eig(A);
lambdasExpected = diag(lambdasExpected);
[lambdaMinExpected,ind] = min(lambdasExpected);
vectorMinExpected = vectorsExpected(:,ind);
% sanity check
if norm(A * vectorMinExpected - lambdaMinExpected * vectorMinExpected ) > 1e-5
   error('wrong expected min') 
end

[Q,R] = qr(A1);
sort(lambdasExpected)
sort(diag(R))

L = chol(A,'lower')
sort(lambdasExpected)
sort(diag(L))
norm(A - L*L')

%     // the following does not work:
%     //    int rank = -1;
%     //    gtsam::Vector x;
%     //    double minEig = numericalUpperBound;
%     //    gtsam::Matrix llt = gtsam::LLt(M); // lower triangular
%     //    for (size_t i = 0; i < llt.rows(); ++i)
%     //      minEig = std::min(minEig,llt(i,i));
%     //    return boost::tuple<int, double, gtsam::Vector>((int)rank, minEig, x);