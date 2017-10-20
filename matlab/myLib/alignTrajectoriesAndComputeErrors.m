function [rotErrors_vio_align, tranErrors_vio_align,Tran_GT,Tran_VIO_aligned] = alignTrajectoriesAndComputeErrors(posesGT,posesVIO,hasGroundTruth)

if nargin < 3
    hasGroundTruth = 1;
end

nrPoses = length(posesGT);
if nrPoses ~= length(posesVIO)
    error('nrPoses mismatch in trajectory alignment')
end
% get translations:
Tran_GT = zeros(nrPoses,3); % keyframes position ground truth
Tran_VIO = zeros(nrPoses,3); % keyframes position estimate
for i=1:nrPoses
    Tran_VIO(i,:) = posesVIO(i).t';
    Tran_GT(i,:) = posesGT(i).t';
end

if hasGroundTruth
    [~,Tran_VIO_aligned,transformation] = procrustes(Tran_GT,Tran_VIO,'scaling',false,'reflection',false);
    R = transformation.T';
    T = transformation.c;
    scale = transformation.b;
    if abs(scale-1)>1e-5
        warning('procrustes messed up with the scale: skipping transformation')
        scale = 1;
        R = eye(3);
        T = zeros(nrPoses,3);
    end
else
    warning('no alignment since there is no ground truth')
    Tran_VIO_aligned = Tran_VIO; % no alignment
    scale = 1;
    R = eye(3);
    T = zeros(nrPoses,3);
end

% get errors (vectors) and make sure that transformation agrees with Tran_VIO_aligned
rotErrors_vio_align = zeros(nrPoses,1); 
tranErrors_vio_align = zeros(nrPoses,1); 
for i=1:nrPoses
    tranErrors_vio_align(i) = norm(Tran_GT(i,:) - Tran_VIO_aligned(i,:)); % translation error
    
    actualT = Tran_VIO_aligned(i,:)';
    expectedT = R * Tran_VIO(i,:)' + T(i,:)';
    if norm(actualT - expectedT)>1e-4
        warning('procrustes transformation mismatch')
    end
    
    R_VIO_aligned = R * posesVIO(i).R;
    Rerr = posesGT(i).R' * R_VIO_aligned;
    cos_theta_err = 0.5 * (trace(Rerr)-1);
    rotErrors_vio_align(i) = abs(acos(cos_theta_err));
end