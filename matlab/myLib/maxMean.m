function [max_x,mean_x]  = maxMean(x)
% works also for matrices, in which case computes stats by columns

max_x = max(x);
mean_x = mean(x);
