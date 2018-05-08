clear all;
close all;
clc

% datasetPath = '/homes/zhangzd/research/slam/data/MH_01_easy';
datasetPath = '/home/luca/data/MH_01_easy';
initialFrameID = 800;
finalFrameID = 2000;
saveImages = 2;
nrRuns = 1;

%% VIO PARAMS
% COMMENT: Irrelevant:
parameterTuningSingle('imuIntegrationSigma',[1e-6 1e-08 1e-10],datasetPath,initialFrameID,finalFrameID,nrRuns,saveImages,'vio')
% COMMENT:
% vioParams.gyroBiasSigma = 1.9393e-05;
% COMMENT:
% vioParams.accBiasSigma = 0.003;
%% STRANGE: error gets larger when this gets smaller? (time instead makes sense)
parameterTuningSingle('relinearizeThreshold',[1e-1 1e-02 1e-3],datasetPath,initialFrameID,finalFrameID,nrRuns,saveImages,'vio')
% COMMENT:
% vioParams.relinearizeSkip = 1;
% AS EXPECTED: error decreases but trend is not very clear
parameterTuningSingle('numOptimize',[1 5 10],datasetPath,initialFrameID,finalFrameID,nrRuns,saveImages,'vio')
% COMMENT: As expected: increasing horizon increase time and reduce errors
parameterTuningSingle('horizon',[5 10 15],datasetPath,initialFrameID,finalFrameID,nrRuns,saveImages,'vio')
%% IMPORTANT: error gets smaller when this is larger
parameterTuningSingle('smartNoiseSigma',[0.1 1 5],datasetPath,initialFrameID,finalFrameID,nrRuns,saveImages,'vio')
%% STRANGE: not a huge impact: average performance suggests to keep this small, but translations suggest otherwise
parameterTuningSingle('rankTolerance',[0.01 0.1 1],datasetPath,initialFrameID,finalFrameID,nrRuns,saveImages,'vio')
%% IMPORTANT: error clearly decreases, hence 1 is too strict, 5 seems good
parameterTuningSingle('outlierRejection',[1 3 5],datasetPath,initialFrameID,finalFrameID,nrRuns,saveImages,'vio')
% COMMENT: no clear trend for error, but timing decreases for increasing threshold
parameterTuningSingle('retriangulationThreshold',[1e-5 1e-3 1e-1],datasetPath,initialFrameID,finalFrameID,nrRuns,saveImages,'vio')

%% Tracker params
% COMMENT: no clear trend
parameterTuningSingle('klt_win_size',[15 21 25],datasetPath,initialFrameID,finalFrameID,nrRuns,saveImages,'tracker')
%% IMPORTANT: large value greatly increase accuracy and strange (bad not too bad) trend on time
parameterTuningSingle('klt_eps',[0.001 0.01 0.1],datasetPath,initialFrameID,finalFrameID,nrRuns,saveImages,'tracker')
% COMMENT:
% trackerParams.maxFeatureAge = 10;
%% STRANGE: absolutely no trend
parameterTuningSingle('maxFeaturesPerFrame',[200 300 400],datasetPath,initialFrameID,finalFrameID,nrRuns,saveImages,'tracker')
%% STRANGE: rotation has no clear trend, but translation suggests to pick this low = 0.2
parameterTuningSingle('quality_level',[0.2 0.4 0.6],datasetPath,initialFrameID,finalFrameID,nrRuns,saveImages,'tracker')
%% IMPORTANT: larger spacing improves errors (and timing?)
parameterTuningSingle('min_distance',[20 40 60],datasetPath,initialFrameID,finalFrameID,nrRuns,saveImages,'tracker')
% COMMENT:
% trackerParams.block_size = 3;
% COMMENT: no clear trend
parameterTuningSingle('ransac_threshold_mono',[1e-8 1e-7 1e-6],datasetPath,initialFrameID,finalFrameID,nrRuns,saveImages,'tracker')
%% STRANGE: absolutely no trend
parameterTuningSingle('ransac_threshold_stereo',[0.1 0.3 0.5],datasetPath,initialFrameID,finalFrameID,nrRuns,saveImages,'tracker')
% COMMENT:
% trackerParams.intra_keyframe_time = 0.2;
% COMMENT:
% trackerParams.minNumberFeatures = 100;
% COMMENT:
% trackerParams.useStereoTracking = true;


% TODO do parameter tuning over the following parameters:
% Type of robust cost function.

% Noise sigma.

% Parameter of loss function.
