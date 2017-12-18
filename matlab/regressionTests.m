function [mean_rotErrors_vio_align, mean_tranErrors_vio_align, run1results] = ...
    regressionTests(trackerParams,vioParams,datasetToRun,testCondition,...
    testCounter, usePlain, useSudo)

if ~exist('usePlain', 'var')
    usePlain = 0;
end
if ~exist('useSudo', 'var')
    useSudo = 1;
end
     
addpath('./myLib/')
if nargin < 1
    close all; clc;  
    vioParams = defaultVioParams();
    trackerParams = defaultTrackerParams();
    datasetToRun = [1]; % first dataset
end

if(length(datasetToRun)>1)
    warning('current regressionTests function is made to run a single test at a time')
end

switch trackerParams.featureSelectionCriterion
    case 0
        selectStr = 'QUALITY';
    case 1
        selectStr = 'EIG_MIN';
    case 2
        selectStr = 'LOG_DET';
    case 3
        selectStr = 'RANDOM';
end
if trackerParams.featureSelectionUseLazyEvaluation==1 && trackerParams.featureSelectionCriterion>0
   selectStr = horzcat(selectStr,'-LAZY') 
end

description = horzcat('minFeat',num2str(trackerParams.minNumberFeatures),'-maxFeat',num2str(trackerParams.maxFeaturesPerFrame),...
    '-stereo',num2str(trackerParams.useStereoTracking),'-kfTime',num2str(10*trackerParams.intra_keyframe_time),'-',selectStr,'-cond',num2str(testCondition),'-test',num2str(testCounter))

% Setting dataPath to different locations depending on the server
fcn_file_path = mfilename('fullpath');
[pathstr, ~, ~] = fileparts(fcn_file_path);
% dataPath = '/home/luca/data/';
dataPath = '/home/luca/data/euroc/';

datasetPaths = {horzcat(dataPath,'MH_01_easy'),horzcat(dataPath,'MH_02_easy'),horzcat(dataPath,'MH_03_medium'),...
    horzcat(dataPath,'MH_04_difficult'),horzcat(dataPath,'MH_05_difficult'), ...
    horzcat(dataPath,'V1_01_easy'),horzcat(dataPath,'V1_02_medium'),horzcat(dataPath,'V1_03_difficult'),...
    horzcat(dataPath,'V2_01_easy'),horzcat(dataPath,'V2_02_medium'),horzcat(dataPath,'V2_03_difficult'),...
    horzcat(dataPath,'killian_01'),horzcat(dataPath,'killian_02'),horzcat(dataPath,'killian_03'),...
    horzcat(dataPath,'B36_01'),horzcat(dataPath,'B38_01'),horzcat(dataPath,'eastman_01')};

saveImages = 2;
nrRuns = 1;
initialFrameID = -1; % use default from cpp
finalFrameID = -1; % use default from cpp

if(trackerParams.maxFeatureAge * trackerParams.intra_keyframe_time + 1 > vioParams.horizon)
    error('horizon is short compared to maxFeatureAge: this might segfault')
end

%% READY TO RUN:
hasGroundTruth = 1;

% params files read by cpp
filenameVioParams = './vioParameters.txt';
filenameTrackerParams = './trackerParameters.txt';
for i = 1:length(datasetToRun)
    datasetId = datasetToRun(i);
    
    %% DATASET SPECIFIC SETTINGS
    %     if datasetId <= 5 % MACHINE HALL: uav moves at the beginning
    %         vioParams.autoInitialize = 0;
    %     end
        if datasetId > 5
            %vioParams.autoInitialize = 1;
            %vioParams.initialRollPitchSigma = 20.0 / 180.0 * pi;
            %vioParams.initialYawSigma  = 3.0 / 180.0 * pi;
            trackerParams.featureSelectionDefaultDepth = 2.0; % indoor scenarios
        end
    %     if datasetId > 11 % duo datasets
    %         disp('Using special parameters for duo dataset')
    %         vioParams.autoInitialize = 1;
    %         vioParams.roundOnAutoInitialize = 1;
    %         trackerParams.maxPointDist = 2; % duo cannot triangulate further
    %         trackerParams.nominalBaseline = 0.03;
    %         vioParams.initialRollPitchSigma = 20.0 / 180.0 * pi;
    %         vioParams.initialYawSigma  = 1.0 / 180.0 * pi;
    %         vioParams.gyroNoiseDensity = 0.0016968; % given by specs (cov_discr = cov_cont/dt, e.g. 100 * cov_cont)
    %         %vioParams.accNoiseDensity = 0.05;  % given by specs
    %         vioParams.accNoiseDensity = 0.05;  % given by specs
    %         vioParams.gyroBiasSigma = 1.9393e-05;
    %         vioParams.accBiasSigma = 0.005;
    %         vioParams.initialAccBiasSigma = 1;
    %         vioParams.initialGyroBiasSigma = 1e-2;
    %         hasGroundTruth = 0;
    %         trackerParams.maxFeaturesPerFrame = 50;
    %         trackerParams.featureSelectionNrCornersToDetect = 50;
    %         trackerParams.ransac_threshold_mono = 1e-5;
    %         trackerParams.ransac_probability = 0.8;
    %         vioParams.horizon = 6/0.2 * trackerParams.intra_keyframe_time;  
    %     end
    % ////////////////////////////////////////////////////////////
    datasetPath = datasetPaths{datasetId};
    datasetName = datasetPath(length(dataPath)+1:end);
    for j = 1:nrRuns
        fprintf('Test %d/%d, run %d/%d (%s)\n',i,length(datasetToRun),j,nrRuns,datasetName)
        removeOutput()

        %% write params and run
        writeVioParamsYAML(filenameVioParams, vioParams);
        writeTrackerParamsYAML(filenameTrackerParams, trackerParams);

        pause(5)
        RUN_STEREO_VIO
        runResults(i,j).results = visualizeResultsVIO('./',saveImages,hasGroundTruth);
        runResults(i,j).vioParams = vioParams;
        runResults(i,j).trackerParams = trackerParams;
        
        mean_rotErrors_vio_align(i,j) = mean(runResults(i,j).results.rotErrors_vio_align);
        mean_tranErrors_vio_align(i,j) = mean(runResults(i,j).results.tranErrors_vio_align);

        randStr =  ''; % horzcat('-rand', num2str(round(10^5 * rand())));
        folderName = horzcat('result-',datasetName,'_run',num2str(j),'_',description,randStr);
        moveOutput(folderName)
        pause(5)
    end
end

if(size(runResults,1)~=1 || size(runResults,1)~=1)
    warning('regression Tests: multiple runs, only one logged!')
end
run1results = runResults(1,1);
run1results.mean_rotErrors_vio_align = mean_rotErrors_vio_align;
run1results.mean_tranErrors_vio_align =mean_tranErrors_vio_align;

%% save results to file
datasetsStr = '';
for i=1:length(datasetToRun)
 datasetsStr = horzcat(datasetsStr,'-',num2str(datasetToRun(i)));
end
save(horzcat('result-regressionTests_',description,datasetsStr,randStr,'.mat'))

