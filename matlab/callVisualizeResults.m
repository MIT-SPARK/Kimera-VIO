% This script calls visualizeResultsVIO to plot the stereoVIO results in a
% certain folder.

clc;
clear;
close all;

% path to the folder
result_path = '~/research/slam/stereoVIO_github/stereoVIO/build_plain/';

% parameters
do_save_figures = 1;
has_ground_truth = 1;

% call the function
visualizeResultsVIO(result_path, do_save_figures, has_ground_truth);