clear all
close all
clc

addpath('./myLib/')
% datasetPath = '/Users/Luca/data/MH_01_easy';
datasetPath = '/home/luca/data/MH_01_easy';
filenameVioParams = './vioParameters.txt';
filenameTrackerParams = './trackerParameters.txt';
initialFrameID = 800;
finalFrameID = 1000;
saveImages = 0;

vioParams = defaultVioParams();
trackerParams = defaultTrackerParams();
writeVioParamsYAML(filenameVioParams, vioParams);
writeTrackerParamsYAML(filenameTrackerParams, trackerParams);

relTol = 1e-1; % 10%
nrTests = 3;
for i=1:nrTests
    RUN_STEREO_VIO
    results(i) = visualizeResultsVIO('./',saveImages);
    disp(results(i))
    
    %% check repeatability
    if i > 1 
       if ~resultsEqual(results(i-1),results(i),relTol)
           warning('VIO pipeline not repeatable!!')
       end
    end
end



