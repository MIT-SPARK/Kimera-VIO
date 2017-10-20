function [t,R] = getTranRotFromLine(rowVector) 
% row vector contains a 3D translation and a 3D rotation listed by rows
% for a total of 12 elements

t = rowVector(1:3)';
R = [rowVector(4:6) 
    rowVector(7:9) 
    rowVector(10:12)];

