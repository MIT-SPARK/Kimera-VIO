clear all
close all
clc

addpath('./myLib/')
nrRuns = 3
nrDatasetsToTest = 11;
allDatasetsToRun = repmat([1:nrDatasetsToTest],1,nrRuns); % nrRuns runs of each datasets

%% CONDITIONS TO TEST: templ_cols (RUNNING)
% trackerCondition = 1;
% conditionName = 'templ_cols'
% conditions = [11 41 71 101];  % in sw is 41, in code is 101
% % RUN!!
% RUN_REGRESSION_TESTS

%% CONDITIONS TO TEST: klt_max_level (RUNNING)
% trackerCondition = 1;
% conditionName = 'klt_max_level'
% conditions = [0 1 2 3 4]; 
% % RUN!!
% RUN_REGRESSION_TESTS

%% CONDITIONS TO TEST: horizon (RUNNING)
% trackerCondition = 0;
% conditionName = 'horizon'
% conditions = [2 4 6 8 10]; % in seconds
% % RUN!!
% RUN_REGRESSION_TESTS

%% CONDITIONS TO TEST: numOptimize (RUNNING)
% trackerCondition = 0;
% conditionName = 'numOptimize'
% conditions = [0  2  4]; 
% % RUN!!
% RUN_REGRESSION_TESTS

%% CONDITIONS TO TEST: maxFeaturesPerFrame (DONE)
% trackerCondition = 1;
% conditionName = 'maxFeaturesPerFrame'
% conditions = [20 50 100 200 300]; 
% % RUN!!
% RUN_REGRESSION_TESTS

%% CONDITIONS TO TEST: klt_max_iter (MAY BE LESS RELEVANT)
% trackerCondition = 1;
% conditionName = 'klt_max_iter'
% conditions = [10 20 30 40 50]; % 30 is default in opencv and our code
% % RUN!!
% RUN_REGRESSION_TESTS

%% CONDITIONS TO TEST: klt_win_size (DONE)
% trackerCondition = 1;
% conditionName = 'klt_win_size'
% conditions = [6 12 18 24 30]; % 21 is default in opencv, our code is 24
% % RUN!!
% RUN_REGRESSION_TESTS

%% CONDITIONS TO TEST: intra_keyframe_time (DONE) 
% trackerCondition = 1;
% conditionName = 'intra_keyframe_time'
% conditions = [0.1 0.2 0.3 0.4 0.5];
% % RUN!!
% RUN_REGRESSION_TESTS

%% CONDITIONS TO TEST: ransac_use_2point_mono (DONE)
% trackerCondition = 1;
% conditionName = 'ransac_use_2point_mono'
% conditions = [0 1];
% % RUN!!
% RUN_REGRESSION_TESTS

%% CONDITIONS TO TEST: ransac_use_1point_stereo (DONE)
% trackerCondition = 1;
% conditionName = 'ransac_use_1point_stereo'
% conditions = [0 1]; 
% % RUN!!
% RUN_REGRESSION_TESTS
 
%% less interesting
% subpixelRefinementStereo
% trackerParams.useStereoTracking
% maxFeatureAge = 10; % 25




