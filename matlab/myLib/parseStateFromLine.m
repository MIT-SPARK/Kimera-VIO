function [timestamp, t, R, vel,imuBiasAcc,imuBiasGyro] = parseStateFromLine(rowVector) 
% row vector contains a keyframeID, a 3D translation and a 3D rotation listed by rows,
% veloty (3 elements), acc bias (3 elements), gyro bias (3 elements)
% for a total of 22 elements

timestamp = rowVector(1);

t = rowVector(2:4)';
R = [rowVector(5:7) 
    rowVector(8:10) 
    rowVector(11:13)];
vel = rowVector(14:16)';
imuBiasAcc = rowVector(17:19)';
imuBiasGyro = rowVector(20:22)';

