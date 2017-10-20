close all
clear all
clc

addpath(genpath('./myLib'))

Rot = eye(3);
tran = [1 2 3]';

ref_frame(Rot,tran)