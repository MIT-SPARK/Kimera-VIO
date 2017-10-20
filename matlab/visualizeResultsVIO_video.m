clear all
close all
clc
addpath('./myLib')

resultsInFolder = {'./result-MH_01_easy_run1_minFeat0-maxFeat200-stereo1-kfTime2-QUALITY-LAZY/',...
    './result-MH_02_easy_run1_minFeat0-maxFeat200-stereo1-kfTime2-QUALITY-LAZY/','./result-FeatureSelection-MH_03_medium-QUALITY/',...
    './result-FeatureSelection-MH_04_difficult-QUALITY/','./result-FeatureSelection-MH_05_difficult-QUALITY/',...
    './result-FeatureSelection-V1_01_easy-QUALITY/','./result-FeatureSelection-V1_02_medium-QUALITY/',...
    './result-FeatureSelection-V1_03_difficult-QUALITY/','./result-FeatureSelection-V2_01_easy-QUALITY/',...
    './result-FeatureSelection-V2_02_medium-QUALITY/','./result-FeatureSelection-V2_03_difficult-QUALITY/'};

framerate  = 20*0.2;

timeLim = Inf;
for itest=1 %:length(resultsInFolder)
resultsOutFolder = horzcat('./',num2str(itest)); % resultsInFolder;

%% create video drone
M_posesVIO = dlmread(horzcat(resultsInFolder{itest},'output_posesVIO.txt'));
if size(M_posesVIO,2)~=22
    error('M_posesVIO:size of data is wrong')
end
M_posesGT = dlmread(horzcat(resultsInFolder{itest},'output_posesGT.txt'));
if size(M_posesGT,2)~=22
    error('M_posesGT:size of data is wrong')
end
nrPoses = size(M_posesVIO,1);
if(size(M_posesGT,1)~= nrPoses)
    error('different number of poses between gt and vio')
end

for i=1:min(timeLim,nrPoses)
    
    [~,posesGT(i).t,posesGT(i).R,velGT(:,i),accBiasGT(:,i),gyroBiasGT(:,i)] = ...
        parseStateFromLine(M_posesGT(i,:));
    [keyframeIDsvio(i),posesVIO(i).t,posesVIO(i).R,velVio(:,i),accBiasVio(:,i),gyroBiasVio(:,i)] = ...
        parseStateFromLine(M_posesVIO(i,:));   
    
    if M_posesVIO(i,1) ~= M_posesGT(i,1)
        error('inconsistent timestamps in VIO and GT poses')
    end
end
createDroneVideo(keyframeIDsvio,posesVIO,posesGT,resultsOutFolder,framerate);
end