clear all
close all
clc

%% GENERAL SETTINGS
minNumberFeatures = 0; % 0 if fixed keyframe rate
maxFeaturesPerFrame = 50;
useStereoTracking = 1; % use stereo
intra_keyframe_time = 0.2;
lazy = 1; 

%% RUN!
%% LOG DET
for datasetToRun = [1:11]
    selector = 2;
    regressionTests(selector,lazy,minNumberFeatures,maxFeaturesPerFrame,...
        useStereoTracking,intra_keyframe_time,datasetToRun)
    
    %% QUALITY
    selector = 0;
    regressionTests(selector,lazy,minNumberFeatures,maxFeaturesPerFrame,...
        useStereoTracking,intra_keyframe_time,datasetToRun)
    
    %% MIN_EIG
    selector = 1;
    regressionTests(selector,lazy,minNumberFeatures,maxFeaturesPerFrame,...
        useStereoTracking,intra_keyframe_time,datasetToRun)
end
