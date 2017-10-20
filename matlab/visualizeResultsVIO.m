function results = visualizeResultsVIO(resultsInFolder,doSaveFigures,hasGroundTruth)
% doSaveFigures = 0: no save nor display
% doSaveFigures = 1: display but no save
% doSaveFigures = 2: save and display
close all
addpath('./myLib')

timeLim = Inf
if nargin<3
    hasGroundTruth = 1;
end
if nargin<2
    doSaveFigures = 2;
end
if doSaveFigures >= 2
    visualizeTrajectory = 1;
else
    visualizeTrajectory = 0;
end
visualizeTrajectory = 0

if nargin < 1
    % resultsInFolder = '../build/';
    resultsInFolder = '../matlab/';
end

resultsOutFolder = resultsInFolder;
M = dlmread(horzcat(resultsInFolder,'output.txt')); 
n = size(M,1);
M = M(1:min(timeLim,n),:);
if size(M,2)~=26
    error('size of data is wrong')
end

%% RANSAC mono info
status_mono = M(:,1);
relRotErrors_mono = M(:,2);
relTranErrors_mono = M(:,3);
nrTrackedFeatures = M(:,4);

%% RANSAC stereo info
status_stereo = M(:,5);
relRotErrors_stereo = M(:,6);
relTranErrors_stereo = M(:,7);
nrTrackedFeatures2 = M(:,8);
if norm(nrTrackedFeatures2-nrTrackedFeatures)>1e-3
    error('nrTrackedFeatures inccent')
end
testError_mono = status_mono / 3; % normalized in 0-1
testError_stereo = status_stereo / 3; % normalized in 0-1

%% VIO info
cur_ids = M(:,9);
rotErrors_vio = M(:,10);
tranErrors_vio = M(:,11);
landmarkCount = M(:,12);
rpy_gt = M(:,13:15);
rpy_vio = M(:,16:18);
relRotErrors_vio = M(:,19);
relTranErrors_vio = M(:,20);
relativeRotError_imu_wrt_gt = M(:,21);
relativeRotError_imu_wrt_5point = M(:,22);
relRotErrors_imuPredict = M(:,23);
relTranErrors_imuPredict = M(:,24);
relTranErrors_stereoRansac = M(:,25);
relTranErrorsMahalanobis_stereoRansac = M(:,26);
if (max(diff(cur_ids)) ~= 1 || min(diff(cur_ids)) ~= 1)
    error('incorrect cur_ids!')
end
if norm(relTranErrors_stereo - relTranErrors_stereoRansac,inf') > 0.001
    warning('incorrect stereo translation error computation!')
end

%% Smart factors info
M_sf = dlmread(horzcat(resultsInFolder,'output_smartFactors.txt')); 
if size(M_sf,2)~=15
    error('size of data is wrong')
end
keyframesId = M_sf(:,1);
framesId = M_sf(:,2);
timestampsSec = M_sf(:,3);
numSF = M_sf(:,4);
numValid = M_sf(:,5);
numDegenerate = M_sf(:,6);
numFarPoints = M_sf(:,7);
numOutliers = M_sf(:,8);
numCheirality = M_sf(:,9);
meanPixelError = M_sf(:,10);
maxPixelError = M_sf(:,11);
meanTrackLength = M_sf(:,12);
maxTrackLength = M_sf(:,13);
nrElementsInMatrix = M_sf(:,14);
nrZeroElementsInMatrix = M_sf(:,15);

% if norm(numSF -numValid-numDegenerate-numFarPoints-numOutliers-numCheirality ) > 1e-4
%     error('something wrong in smart factors count')
% end
if norm(cur_ids - keyframesId) > 1e-4
    error('keyframesId mismatch');
end

%% plot keyframe spacing (not regularly spaced in time now)
if (doSaveFigures>=1)
    fh = figure;
    subplot(1,2,1); hold on
    plot(keyframesId,framesId,'-b')
    axis equal
    xlabel('keyframes')
    ylabel('frame id')
    title('keyframe-frame spacing')
    %
    subplot(1,2,2); hold on
    plot(keyframesId,timestampsSec-timestampsSec(1),'-b')
    xlabel('keyframe')
    ylabel('time')
    title('keyframe-time spacing')
end
if (doSaveFigures>=2)
    filename = horzcat(resultsOutFolder,'/keyframeSpacing');
    saveas(fh,filename,'epsc'); saveas(fh,filename,'fig');
end

%% consistency check for stereo translation estimates
if (doSaveFigures>=1)
    fh = figure; hold on
    semilogy(keyframesId,log10(relTranErrorsMahalanobis_stereoRansac),'-b')
    xlabel('keyframes')
    ylabel('stereo translation Consistency (log mahl dist)')
end
if (doSaveFigures>=2)
    filename = horzcat(resultsOutFolder,'/stereoTraslationConsistency');
    saveas(fh,filename,'epsc'); saveas(fh,filename,'fig');
end

%% plot estimation errors mono
if (doSaveFigures>=1)
    fh = plotErrorWithStatus(relRotErrors_mono,relTranErrors_mono,testError_mono,'mono');
end
if (doSaveFigures>=2)
    filename = horzcat(resultsOutFolder,'/monoErrors');
    saveas(fh,filename,'epsc'); saveas(fh,filename,'fig');
end

%% plot estimation errors imu preintegration
if (doSaveFigures>=1)
    fh = figure;
    subplot(1,2,1); hold on;
    plot(relativeRotError_imu_wrt_gt,'-g','linewidth',2)
    title('relativeRotError imu wrt gt')
    subplot(1,2,2); hold on;
    plot(relativeRotError_imu_wrt_5point,'-r','linewidth',2)
    title('relativeRotError imu wrt 5point')
end
if (doSaveFigures>=2)
    filename = horzcat(resultsOutFolder,'/imuRotPreintErrors');
    saveas(fh,filename,'epsc'); saveas(fh,filename,'fig');
end

%% plot estimation errors stereo
if (doSaveFigures>=1)
    fh = plotErrorWithStatus(relRotErrors_stereo,relTranErrors_stereo,testError_stereo,'stereo');
end
if (doSaveFigures>=2)
    filename = horzcat(resultsOutFolder,'/stereoErrors');
    saveas(fh,filename,'epsc'); saveas(fh,filename,'fig');
end

%% plot nr of tracker features
if (doSaveFigures>=1)
    fh = figure; hold on
    title('nr of tracked features')
    plot(nrTrackedFeatures,'-k','linewidth',2);
    xlabel('keyframes'); ylabel('nr features')
end
if (doSaveFigures>=2)
    filename = horzcat(resultsOutFolder,'/nrTrackedFeatures');
    saveas(fh,filename,'epsc'); saveas(fh,filename,'fig');
end

%% plot vio errors
if (doSaveFigures>=1)
    fh =  plotErrorWithStatus(rotErrors_vio,tranErrors_vio,zeros(size(rotErrors_vio)),'vio');
end
if (doSaveFigures>=2)
    filename = horzcat(resultsOutFolder,'/vioErrors');
    saveas(fh,filename,'epsc'); saveas(fh,filename,'fig');
end

%% plot vio RELATIVE errors
if (doSaveFigures>=1)
    fh =  plotErrorWithStatus(relRotErrors_vio,relTranErrors_vio,zeros(size(rotErrors_vio)),'vio (relative)');
end
if (doSaveFigures>=2)
    filename = horzcat(resultsOutFolder,'/vioRelativeErrors');
    saveas(fh,filename,'epsc'); saveas(fh,filename,'fig');
end

%% plot imuPredict (RELATIVE) errors
if (doSaveFigures>=1)
    fh =  plotErrorWithStatus(relRotErrors_imuPredict,relTranErrors_imuPredict,zeros(size(relRotErrors_imuPredict)),'imuPredict (relative)');
end
if (doSaveFigures>=2)
    filename = horzcat(resultsOutFolder,'/imuPredictRelativeErrors');
    saveas(fh,filename,'epsc'); saveas(fh,filename,'fig');
end

%% plot rpy errors
err_rpy = rpy_gt - rpy_vio;
if (doSaveFigures>=1)
    fh = figure;
    subplot(1,2,1); hold on; title('yaw errors')
    plot(wrapToPi(err_rpy(:,3)),'-b')
    subplot(1,2,2); hold on; title('roll-pitch errors')
    plot(wrapToPi(err_rpy(:,1)),'-r')
    plot(wrapToPi(err_rpy(:,2)),'-g')
    legend('roll','pitch')
end
if (doSaveFigures>=2)
    filename = horzcat(resultsOutFolder,'/rpyErrors');
    saveas(fh,filename,'epsc'); saveas(fh,filename,'fig');
end

%% plot smart factors statistics:
if (doSaveFigures>=1)
    fh = figure; hold on
    title('smart factors statistics')
    xlabel('keyframes')
    plot(numSF,'-k')
    plot(numValid,'-g')
    plot(numDegenerate,'-r')
    plot(numFarPoints,'--k')
    plot(numOutliers,'--k')
    plot(numCheirality,'-m')
    legend('numSF','numValid','numDegenerate','numFarPoints','numOutliers','numCheirality')
end
if (doSaveFigures>=2)
    filename = horzcat(resultsOutFolder,'/smartFactorsStats');
    saveas(fh,filename,'epsc'); saveas(fh,filename,'fig');
end

%% feature tracks stats
if (doSaveFigures>=1)
    fh = figure; hold on
    title('smart factors statistics')
    xlabel('keyframes')
    plot(maxTrackLength,'-k')
    plot(meanTrackLength,'--k')
    legend('maxTrackLength','meanTrackLength')
end
if (doSaveFigures>=2)
    filename = horzcat(resultsOutFolder,'/trackLengthStats');
    saveas(fh,filename,'epsc'); saveas(fh,filename,'fig');
end

%% create video drone
disp('- reading poses')
M_posesVIO = dlmread(horzcat(resultsInFolder,'output_posesVIO.txt'));
if size(M_posesVIO,2)~=22
    error('M_posesVIO:size of data is wrong')
end
M_posesGT = dlmread(horzcat(resultsInFolder,'output_posesGT.txt'));
if size(M_posesGT,2)~=22
    error('M_posesGT:size of data is wrong')
end
nrPoses = size(M_posesVIO,1);
if(size(M_posesGT,1)~= nrPoses)
    error('different number of poses between gt and vio')
end

for i=1:min(timeLim,nrPoses)
    [keyframeIDsvio(i),posesVIO(i).t,posesVIO(i).R,velVio(:,i),accBiasVio(:,i),gyroBiasVio(:,i)] = ...
        parseStateFromLine(M_posesVIO(i,:));
    [~,posesGT(i).t,posesGT(i).R,velGT(:,i),accBiasGT(:,i),gyroBiasGT(:,i)] = ...
        parseStateFromLine(M_posesGT(i,:));
    if M_posesVIO(i,1) ~= M_posesGT(i,1)
        error('inconsistent timestamps in VIO and GT poses')
    end
end
if(visualizeTrajectory)
    createDroneVideo(keyframeIDsvio,posesVIO,posesGT,resultsOutFolder);
end

%% visualize aligned trajectories
disp('- aligning trajectories')
[rotErrors_vio_align, tranErrors_vio_align,Tran_GT,Tran_VIO_aligned] = alignTrajectoriesAndComputeErrors(posesGT,posesVIO,hasGroundTruth);
% plot errors after alignment
if (doSaveFigures>=1)
    fh = plotErrorWithStatus(rotErrors_vio_align,tranErrors_vio_align,zeros(size(rotErrors_vio_align)),'vioAligned');
end
if (doSaveFigures>=2)
    filename = horzcat(resultsOutFolder,'/vioErrorsAligned');
    saveas(fh,filename,'epsc'); saveas(fh,filename,'fig');
end
% plot trajectories after alignment
if (doSaveFigures>=1)
    [fh] = plotDroneTrajectories(Tran_GT,Tran_VIO_aligned);
end
if (doSaveFigures>=2)
    filename = horzcat(resultsOutFolder,'/vioTrajectoriesAligned');
    saveas(fh,filename,'epsc'); saveas(fh,filename,'fig');
end


%%  display velocity errors
velErrors = velVio - velGT; % 3 x nrPoses matrix
if (doSaveFigures>=1)
    fh = figure; hold on
    normVelErrors = sqrt(velErrors(1,:).^2 + velErrors(2,:).^2 + velErrors(3,:).^2);
    subplot(2,2,1)
    plot(velErrors(1,:),'-r'); hold on; ylabel('x velocity error [m]'); xlabel('keyframes')
    subplot(2,2,2)
    plot(velErrors(2,:),'-g'); hold on; ylabel('y velocity error [m]'); xlabel('keyframes')
    subplot(2,2,3)
    plot(velErrors(3,:),'-b'); hold on; ylabel('z velocity error [m]'); xlabel('keyframes')
    subplot(2,2,4)
    plot(normVelErrors,'-k'); hold on; ylabel('norm velocity error [m]'); xlabel('keyframes')
end
if (doSaveFigures>=2)
    filename = horzcat(resultsOutFolder,'/velocityErrors');
    saveas(fh,filename,'epsc'); saveas(fh,filename,'fig');
end

%% display biases
if (doSaveFigures>=1)
    fh = figure; hold on
    subplot(3,2,1)
    plot(accBiasVio(1,:),'-b'); hold on
    plot(accBiasGT(1,:),'-g');
    ylabel('x bias acc error'); xlabel('keyframes'); legend('vio','gt')
    %
    subplot(3,2,3)
    plot(accBiasVio(2,:),'-b'); hold on
    plot(accBiasGT(2,:),'-g');
    ylabel('y bias acc error'); xlabel('keyframes');
    %
    subplot(3,2,5)
    plot(accBiasVio(3,:),'-b'); hold on
    plot(accBiasGT(3,:),'-g');
    ylabel('z bias acc error'); xlabel('keyframes');
    %
    subplot(3,2,2)
    plot(gyroBiasVio(2,:),'-b'); hold on
    plot(gyroBiasGT(2,:),'-g');
    ylabel('y bias gyro error'); xlabel('keyframes');
    %
    subplot(3,2,4)
    plot(gyroBiasVio(2,:),'-b'); hold on
    plot(gyroBiasGT(2,:),'-g');
    ylabel('y bias gyro error'); xlabel('keyframes');
    %
    subplot(3,2,6)
    plot(gyroBiasVio(2,:),'-b'); hold on
    plot(gyroBiasGT(2,:),'-g');
    ylabel('y bias gyro error'); xlabel('keyframes');
end
if (doSaveFigures>=2)
    filename = horzcat(resultsOutFolder,'/imuBiases');
    saveas(fh,filename,'epsc'); saveas(fh,filename,'fig');
end

%% plot VIO timing
disp('- displaying vio timing')
M_timeVIO = dlmread(horzcat(resultsInFolder,'output_timingVIO.txt'));
if(size(M_timeVIO,2)~=17 || size(M_timeVIO,1)~=nrPoses) error('wrong size of M_timeVIO'); end
% first entry is the keyframeID
keyframeIDs = M_timeVIO(:,1);
factorsAndSlotsTimes = M_timeVIO(:,2);
preUpdateTimes = M_timeVIO(:,3);
updateTimes = M_timeVIO(:,4);
updateSlotTimes = M_timeVIO(:,5);
extraIterationsTimes = M_timeVIO(:,6);
printTimes = M_timeVIO(:,7);

loadStereoFrame_times = M_timeVIO(:,8);
processStereoFrame_times = M_timeVIO(:,9);
featureSelection_times = M_timeVIO(:,10);
overallVIO_times = M_timeVIO(:,11);
linearizeTime = M_timeVIO(:,12);
linearSolveTime = M_timeVIO(:,13);
retractTime = M_timeVIO(:,14);
linearizeMarginalizeTime = M_timeVIO(:,15);
marginalizeTime = M_timeVIO(:,16);
imuPreintegrationTime = M_timeVIO(:,17);

overall_times = loadStereoFrame_times + processStereoFrame_times + featureSelection_times + overallVIO_times;
if (doSaveFigures>=1)
    fh = figure;
    subplot(1,2,1); hold on
    plot(factorsAndSlotsTimes,'-r','linewidth',2)
    plot(preUpdateTimes,'-g','linewidth',2)
    plot(updateTimes,'-b','linewidth',2)
    plot(updateSlotTimes,'-m','linewidth',2)
    plot(extraIterationsTimes,'-c','linewidth',2)
    plot(printTimes,'-k','linewidth',2)
    ylabel('time [sec]');
    xlabel('keyframes');
    title('detailed VIO timing')
    legend('factorsAndSlotsTimes','preUpdateTimes','updateTimes',...
        'updateSlotTimes','extraIterationsTimes','printTimes')
    
    subplot(1,2,2); hold on
    plot(loadStereoFrame_times,'-g','linewidth',2)
    plot(processStereoFrame_times,'-r','linewidth',2)
    plot(featureSelection_times,'-m','linewidth',2)
    plot(overallVIO_times,'-b','linewidth',2)
    plot(overall_times,'-k','linewidth',2)
    ylabel('time [sec]');
    xlabel('keyframes');
    title('overall VIO timing')
    legend('loadStereoFrameTimes','processStereoFrameTimes','featureSelectionTimes','overallVIOTimes','overallTimes')
    
    if (doSaveFigures>=2)
        filename = horzcat(resultsOutFolder,'/vioTiming');
        saveas(fh,filename,'epsc'); saveas(fh,filename,'fig');
    end
    
    fh = figure;
    subplot(1,2,1); hold on
    plot(linearizeTime,'-r','linewidth',2)
    plot(linearSolveTime,'-g','linewidth',2)
    plot(retractTime,'-b','linewidth',2)
    plot(linearizeMarginalizeTime,'-m','linewidth',2)
    plot(marginalizeTime,'-c','linewidth',2)
    plot(imuPreintegrationTime,'-y','linewidth',2)
    plot(overallVIO_times,'-k','linewidth',2)
    ylabel('time [sec]');
    xlabel('keyframes');
    title('VIO timing breakdown')
    legend('linearizeTime','linearSolveTime','retractTime',...
        'linearizeMarginalizeTime','marginalizeTime','imuPreintegrationTime','total')
    
    subplot(1,2,2); hold on
    plot(linearizeTime / overallVIO_times * 100,'-g','linewidth',2)
    plot(linearSolveTime / overallVIO_times * 100,'-r','linewidth',2)
    plot(retractTime / overallVIO_times * 100,'-m','linewidth',2)
    plot(linearizeMarginalizeTime / overallVIO_times * 100,'-b','linewidth',2)
    plot(marginalizeTime / overallVIO_times * 100,'-k','linewidth',2)
    plot(imuPreintegrationTime / overallVIO_times * 100,'-y','linewidth',2)
    ylabel('time [sec] - percentage');
    xlabel('keyframes');
    title('VIO timing breakdown - percent')
    legend('linearizeTime','linearSolveTime','retractTime',...
        'linearizeMarginalizeTime','imuPreintegrationTime','marginalizeTime')
    
    if (doSaveFigures>=2)
        filename = horzcat(resultsOutFolder,'/vioTimingBreakdown');
        saveas(fh,filename,'epsc'); saveas(fh,filename,'fig');
    end 
end

%% plot StereoTracker timing
disp('- displaying StereoTracker timing')
M_timeST = dlmread(horzcat(resultsInFolder,'output_timingTracker.txt'));
if(size(M_timeST,2)~=8 || size(M_timeST,1)~=nrPoses) error('wrong size of M_timeST'); end
% first entry is the keyframeID
featureDetectionTimes = M_timeST(:,2);
featureTrackingTimes = M_timeST(:,3);
monoRansacTimes = M_timeST(:,4);
stereoRansacTimes = M_timeST(:,5);
monoRansacIters = M_timeST(:,6);
stereoRansacIters = M_timeST(:,7);
featureSelectionTimes = M_timeST(:,8);

if (doSaveFigures>=1)
    fh = figure;
    subplot(2,2,1)
    plot(featureDetectionTimes,'-r','linewidth',2); hold on
    plot(featureTrackingTimes,'-g','linewidth',2)
    plot(monoRansacTimes,'-b','linewidth',2)
    plot(stereoRansacTimes,'-m','linewidth',2)
    ylabel('time [sec]');
    xlabel('keyframes');
    title('Stereo Tracker timing')
    legend('featureDetectionTimes','featureTrackingTimes', ...
        'monoRansacTimes','stereoRansacTimes')
    subplot(2,2,3)
    plot(monoRansacIters,'-b','linewidth',2); hold on
    plot(stereoRansacIters,'-m','linewidth',2)
    ylabel('ransac iterations');
    xlabel('keyframes');
    legend('mono','stereo')
    subplot(2,2,2)
    plot(featureSelectionTimes,'-b','linewidth',2); hold on
    ylabel('feature selection time');
    xlabel('keyframes');
    
    subplot(2,2,4)
    plot(nrZeroElementsInMatrix./nrElementsInMatrix,'-b','linewidth',2); hold on
    str = horzcat('percent of zeros, ave nr rows:',num2str(sqrt(mean(nrElementsInMatrix))));
    ylabel(str);
    xlabel('keyframes');
end
if (doSaveFigures>=2)
    filename = horzcat(resultsOutFolder,'/trackerTiming');
    saveas(fh,filename,'epsc'); saveas(fh,filename,'fig');
end

if (doSaveFigures>=1)
    fh = figure; hold on
    plot(featureSelectionTimes,'-m','linewidth',2)
    plot(overallVIO_times,'-b','linewidth',2)
    ylabel('time [sec]');
    xlabel('keyframes');
    str = horzcat('vio (',num2str(mean(overallVIO_times(2:end))),...
        ') + actual feat sel (',num2str(mean(featureSelectionTimes(2:end))),')');
    title(str)
    legend('featureSelection times','overallVIO times')
    
    if (doSaveFigures>=2)
        filename = horzcat(resultsOutFolder,'/featureSelectionTime');
        saveas(fh,filename,'epsc'); saveas(fh,filename,'fig');
    end
end

%% plot StereoTracker statistics
disp('- displaying StereoTracker statistics')
M_statsST = dlmread(horzcat(resultsInFolder,'output_statsTracker.txt'));
if(size(M_statsST,2)~=14 || size(M_statsST,1)~=nrPoses) error('wrong size of M_statsST'); end
% first entry is the keyframeID
nrDetectedFeatures = M_statsST(:,2);
nrTrackerFeatures = M_statsST(:,3);
nrMonoInliers = M_statsST(:,4);
nrMonoPutatives = M_statsST(:,5);
nrStereoInliers = M_statsST(:,6);
nrStereoPutatives = M_statsST(:,7);
nrValidRKP = M_statsST(:,8);
nrNoLeftRectRKP = M_statsST(:,9);
nrNoRightRectRKP = M_statsST(:,10);
nrNoDepthRKP = M_statsST(:,11);
nrFailedArunRKP = M_statsST(:,12);
need_n_corners = M_statsST(:,13);
extracted_corners = M_statsST(:,14);

lw = 1; % line width
%% TRACKER STATS
if (doSaveFigures>=1)
    fh = figure;
    subplot(2,2,1); hold on
    plot(nrDetectedFeatures,'-r','linewidth',lw)
    plot(nrTrackerFeatures,'-g','linewidth',lw)
    ylabel('nr features'); xlabel('keyframes');
    title('Stereo Tracker statistics')
    legend('nrDetectedFeatures','nrTrackerFeatures')
    %
    subplot(2,2,2); hold on
    plot(nrValidRKP,'-g','linewidth',lw)
    plot(nrNoLeftRectRKP,'-r','linewidth',lw)
    plot(nrNoRightRectRKP,'-b','linewidth',lw)
    plot(nrNoDepthRKP,'-m','linewidth',lw)
    plot(nrFailedArunRKP,'--c','linewidth',lw)
    ylabel('nr features with status'); xlabel('keyframes');
    title('Sparse stereo matching statistics (arun = 0 after detection)')
    legend('nrValidRKP','nrNoLeftRectRKP','nrNoRightRectRKP', ...
        'nrNoDepthRKP','nrFailedArunRKP')
    %
    subplot(2,2,3); hold on
    plot(need_n_corners,'-c','linewidth',lw)
    plot(extracted_corners,'-k','linewidth',lw)
    ylabel('nr features');
    xlabel('keyframes');
    title('feature selection statistics')
    legend('needNcorners','extractedCorners')
end
if (doSaveFigures>=2)
    filename = horzcat(resultsOutFolder,'/trackerStats');
    saveas(fh,filename,'epsc'); saveas(fh,filename,'fig');
end

%% RANSAC STATS
if (doSaveFigures>=1)
    fh = figure;
    subplot(1,2,1); hold on
    plot(nrMonoInliers,'-b','linewidth',lw)
    plot(nrMonoPutatives,'-m','linewidth',lw)
    ylabel('nr features'); xlabel('keyframes');
    title('Mono ransac statistics')
    legend('nrMonoInliers','nrMonoPutatives')
    %
    subplot(1,2,2); hold on
    plot(nrStereoInliers,'-c','linewidth',lw)
    plot(nrStereoPutatives,'-k','linewidth',lw)
    ylabel('nr features');
    xlabel('keyframes');
    title('Stereo ransac statistics')
    legend('nrStereoInliers','nrStereoPutatives')
end
if (doSaveFigures>=2)
    filename = horzcat(resultsOutFolder,'/ransacStats');
    saveas(fh,filename,'epsc'); saveas(fh,filename,'fig');
end

%% plot StereoTracker statistics
disp('- display stereo factors stats')
M_statsF = dlmread(horzcat(resultsInFolder,'output_statsFactors.txt'));
if(size(M_statsF,2)~=8 || size(M_statsF,1)~=nrPoses) error('wrong size of M_statsF'); end
% first entry is the keyframeID
numAddedSmartF = M_statsF(:,2);
numAddedImuF = M_statsF(:,3);
numAddedNoMotionF = M_statsF(:,4);
numAddedConstantVelF = M_statsF(:,5);
numAddedBetweenStereoF = M_statsF(:,6);
numKeysInState = M_statsF(:,7);
expectedNumKeysInState = M_statsF(:,8);

if norm(numKeysInState - expectedNumKeysInState) > 1e-3
   warning('==============Mismatch in expected and actual number of states!===========') 
   mismatch = numKeysInState - expectedNumKeysInState;
   ind = find(mismatch > 0);
   [numKeysInState(ind) expectedNumKeysInState(ind)]
end

if (doSaveFigures>=1)
    fh = figure; hold on; title('nr of added factors')
    plot(numAddedSmartF/100,'-r')
    plot(numAddedImuF,'-g')
    plot(numAddedNoMotionF,'-b')
    plot(numAddedConstantVelF,'-m')
    plot(numAddedBetweenStereoF,'-c')
    legend('numAddedSmartF/100','numAddedImuF','numAddedNoMotionF','numAddedConstantVelF','numAddedBetweenStereoF')
    ylabel('added factors'); xlabel('keyframes')
end
if (doSaveFigures>=2)
    filename = horzcat(resultsOutFolder,'/addedFactors');
    saveas(fh,filename,'epsc'); saveas(fh,filename,'fig');
end

if (doSaveFigures>=1)
    fh = figure; hold on; title('nr of keys in state')
    plot(numKeysInState,'-k')
    plot(expectedNumKeysInState,'--r')
    ylabel('nr keys in state'); xlabel('keyframes')
end
if (doSaveFigures>=2)
    filename = horzcat(resultsOutFolder,'/nrKeysInState');
    saveas(fh,filename,'epsc'); saveas(fh,filename,'fig');
end

%% store high level summary
disp('- storing results')
results.nrKeyframes = keyframeIDs(end);

%% Errors
[results.maxRelRotErrors_mono,results.meanRelRotErrors_mono]  = maxMean(relRotErrors_mono(relRotErrors_mono>0));
[results.maxRelTranErrors_mono,results.meanRelTranErrors_mono]  = maxMean(relTranErrors_mono(relTranErrors_mono>0));

[results.maxRelRotErrors_stereo,results.meanRelRotErrors_stereo]  = maxMean(relRotErrors_stereo(relRotErrors_stereo>0));
[results.maxRelTranErrors_stereo,results.meanRelTranErrors_stereo]  = maxMean(relTranErrors_stereo(relTranErrors_stereo>0));

[results.maxRotErrors_vio,results.meanRotErrors_vio]  = maxMean(rotErrors_vio);
[results.maxTranErrors_vio,results.meanTranErrors_vio]  = maxMean(tranErrors_vio);

[results.maxRelRotErrors_vio,results.meanRelRotErrors_vio]  = maxMean(relRotErrors_vio);
[results.maxRelTranErrors_vio,results.meanRelTranErrors_vio]  = maxMean(relTranErrors_vio);

[results.maxNrTrackedFeatures,results.meanNrTrackedFeatures]  = maxMean(nrTrackedFeatures);

[results.maxErr_rpy,results.meanErr_rpy]  = maxMean(err_rpy);

[results.maxVelErrors,results.meanVelErrors]  = maxMean(velErrors');

%% Timing
[results.maxTimeUpdate,results.meanTimeUpdate]  = maxMean(updateTimes);
[results.maxTimeExtraUpdates,results.meanTimeExtraUpdates]  = maxMean(extraIterationsTimes);

[results.maxMonoRansacTimes,results.meanMonoRansacTimes]  = maxMean(monoRansacTimes);
[results.maxStereoRansacTimes,results.meanStereoRansacTimes]  = maxMean(stereoRansacTimes);
[results.maxFeatureDetectionTimes,results.meanFeatureDetectionTimes]  = maxMean(featureDetectionTimes);
[results.maxFeatureTrackingTimes,results.meanFeatureTrackingTimes]  = maxMean(featureTrackingTimes);

%% Detailed results for plotting
results.status_mono = status_mono;
results.relRotErrors_mono = relRotErrors_mono;
results.relTranErrors_mono = relTranErrors_mono;
results.nrTrackedFeatures = nrTrackedFeatures;

results.status_stereo = status_stereo;
results.relRotErrors_stereo = relRotErrors_stereo;
results.relTranErrors_stereo = relTranErrors_stereo;
% nrTrackedFeatures2

% cur_ids 
results.rotErrors_vio = rotErrors_vio;
results.tranErrors_vio = tranErrors_vio;
results.landmarkCount = landmarkCount;
results.relRotErrors_vio = relRotErrors_vio;
results.relTranErrors_vio = relTranErrors_vio;

results.relativeRotError_imu_wrt_gt = relativeRotError_imu_wrt_gt;
results.relativeRotError_imu_wrt_5point = relativeRotError_imu_wrt_5point;
results.relRotErrors_imuPredict = relRotErrors_imuPredict;
results.relTranErrors_imuPredict = relTranErrors_imuPredict;
results.relTranErrors_stereoRansac = relTranErrors_stereoRansac;
results.relTranErrorsMahalanobis_stereoRansac = relTranErrorsMahalanobis_stereoRansac;

%keyframesId
results.framesId = framesId;
results.timestampsSec = timestampsSec;

results.numSF = numSF;
results.numValid = numValid;
results.numDegenerate = numDegenerate;
results.numFarPoints = numFarPoints;
results.numOutliers = numOutliers;
results.numCheirality = numCheirality;
results.meanPixelError = meanPixelError;
results.maxPixelError = maxPixelError;
results.meanTrackLength = meanTrackLength;
results.maxTrackLength = maxTrackLength;
results.nrElementsInMatrix = nrElementsInMatrix;
results.nrZeroElementsInMatrix = nrZeroElementsInMatrix;

results.rotErrors_vio_align = rotErrors_vio_align;
results.tranErrors_vio_align = tranErrors_vio_align;

results.loadStereoFrame_times = loadStereoFrame_times;
results.processStereoFrame_times = processStereoFrame_times;
results.featureSelection_times = featureSelection_times;
results.overallVIO_times = overallVIO_times;
results.overall_times = overall_times;

results.featureSelectionTimes = featureSelectionTimes;
results.normVelErrors =  normVelErrors;

results.keyframeIDs =  keyframeIDs;
results.factorsAndSlotsTimes =  factorsAndSlotsTimes;
results.preUpdateTimes =  preUpdateTimes;
results.updateTimes =  updateTimes;
results.updateSlotTimes =  updateSlotTimes;
results.updateSlotTimes =  updateSlotTimes;
results.extraIterationsTimes =  extraIterationsTimes;
results.printTimes =  printTimes;

results.featureDetectionTimes = featureDetectionTimes ;
results.featureTrackingTimes =  featureTrackingTimes;
results.monoRansacTimes =  monoRansacTimes;
results.stereoRansacTimes =  stereoRansacTimes;
results.monoRansacIters =  monoRansacIters;
results.stereoRansacIters =  stereoRansacIters;
results.featureSelectionTimes = featureSelectionTimes;
results.linearizeTime = linearizeTime;
results.linearSolveTime = linearSolveTime;
results.retractTime = retractTime;
results.linearizeMarginalizeTime = linearizeMarginalizeTime;
results.marginalizeTime = marginalizeTime;
results.imuPreintegrationTime = imuPreintegrationTime;

results.nrDetectedFeatures =  nrDetectedFeatures;
results.nrTrackerFeatures =  nrTrackerFeatures;
results.nrMonoInliers =  nrMonoInliers;
results.nrMonoPutatives = nrMonoPutatives;
results.nrStereoInliers =  nrStereoInliers;
results.nrStereoPutatives = nrStereoPutatives;
results.nrValidRKP = nrValidRKP;
results.nrNoLeftRectRKP =  nrNoLeftRectRKP;
results.nrNoRightRectRKP =  nrNoRightRectRKP;
results.nrNoDepthRKP =  nrNoDepthRKP;
results.nrFailedArunRKP =  nrFailedArunRKP;
results.need_n_corners =  need_n_corners;
results.extracted_corners =  extracted_corners;

results.numAddedSmartF =  numAddedSmartF;
results.numAddedImuF =  numAddedImuF;
results.numAddedNoMotionF =  numAddedNoMotionF;
results.numAddedConstantVelF =  numAddedConstantVelF;
results.numAddedBetweenStereoF =  numAddedBetweenStereoF;

fprintf('ABSOLUTE: Mean rot error: %g, Max rot error: %g, Mean tran error: %g, Max tran error: %g\n', ...
    results.meanRotErrors_vio,results.maxRotErrors_vio,results.meanTranErrors_vio,results.maxTranErrors_vio);

fprintf('RELATIVE: Mean rot error: %g, Max rot error: %g, Mean tran error: %g, Max tran error: %g\n', ...
    results.meanRelRotErrors_vio,results.maxRelRotErrors_vio,results.meanRelTranErrors_vio,results.maxRelTranErrors_vio);










 