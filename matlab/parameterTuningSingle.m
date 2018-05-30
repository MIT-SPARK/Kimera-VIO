function [] = parameterTuningSingle(paramName,conditions,datasetPath,initialFrameID,finalFrameID,nrRuns,saveImages,whichParams)

if nargin < 1
    datasetPath = '/Users/Luca/data/MH_01_easy';
    conditions = [1e-6  1e-08 1e-10];
    paramName = 'imuIntegrationSigma';
    initialFrameID = 800;
    finalFrameID = 900;
    saveImages = 0;
    nrRuns = 2;
end

addpath('./myLib/')
filenameVioParams = './vioParameters.txt';
filenameTrackerParams = './trackerParameters.txt';

regularVio = true;
if (regularVio) 
    vioParams = defaultVioParamsRegularVio();
    trackerParams = defaultTrackerParamsRegularVio();
else
    vioParams = defaultVioParams();
    trackerParams = defaultTrackerParams();
end

if(trackerParams.maxFeatureAge * trackerParams.intra_keyframe_time + 1 > vioParams.horizon)
    error('horizon is short compared to maxFeatureAge: this might segfault')
end

%% VIO params
for i = 1:length(conditions)
    for j = 1:nrRuns
        fprintf('Test %d/%d, run %d/%d (%s params)\n',i,length(conditions),j,nrRuns,whichParams)
        
        switch whichParams
            case 'vio'
                vioParams = setfield(vioParams,paramName,conditions(i));
            case 'tracker'
                trackerParams = setfield(trackerParams,paramName,conditions(i));
            otherwise
                error('wrong choice of whichParams')
        end
        
        %% write params and run
        writeVioParamsYAML(filenameVioParams, vioParams);
        writeTrackerParamsYAML(filenameTrackerParams, trackerParams);
        removeOutput()

        pause(5)
        vioParams, trackerParams
        RUN_STEREO_VIO
        runResults(i,j).results = visualizeResultsVIO('./',saveImages);
        runResults(i,j).vioParams = vioParams;
        runResults(i,j).trackerParams = trackerParams;
        folderName = horzcat('result-',paramName,'_',num2str(conditions(i)),'_run',num2str(j));
        moveOutput(folderName)
        pause(5)
    end
end

close all
%% figure: error
fh = figure();
% Plot BoxPlots instead...
if (saveImages>=1)
    r = 3;
    c = 2;
    subplotCondition(r,c,1,paramName,runResults,conditions,nrRuns,'mean_rotErrors_vio_align','meanRotAligned')
    subplotCondition(r,c,3,paramName,runResults,conditions,nrRuns,'max_rotErrors_vio_align','maxRotAligned')
    subplotCondition(r,c,5,paramName,runResults,conditions,nrRuns,'min_rotErrors_vio_align','minRotAligned')
    subplotCondition(r,c,2,paramName,runResults,conditions,nrRuns,'mean_tranErrors_vio_align','meanTransAligned')
    subplotCondition(r,c,4,paramName,runResults,conditions,nrRuns,'max_tranErrors_vio_align','maxTranAligned')
    subplotCondition(r,c,6,paramName,runResults,conditions,nrRuns,'min_tranErrors_vio_align','minTranAligned')
end
if (saveImages>=2)
    filename = horzcat('resultsParam_',paramName,'_errors');
    print(fh, filename, '-dsvg')
    saveas(fh,filename,'svg');
end

%% figure: time
% if (saveImages>=1)
%     fh = figure();
%     set(fh, 'Visible', 'off');
%     subplotCondition(2,2,1,paramName,runResults,conditions,nrRuns,'meanTimeUpdate')
%     subplotCondition(2,2,3,paramName,runResults,conditions,nrRuns,'maxTimeUpdate')
%     subplotCondition(2,2,2,paramName,runResults,conditions,nrRuns,'meanTimeExtraUpdates')
%     subplotCondition(2,2,4,paramName,runResults,conditions,nrRuns,'maxTimeExtraUpdates')
% end
% if (saveImages>=2)
%     filename = horzcat('resultsParam_',paramName,'_timing');
%     saveas(fh,filename,'epsc');
% end

save(horzcat('resultsParam_',paramName,'.mat'))

folderName = horzcat('results_parameterTuning');
moveOutput(folderName,{'*.mat','*.eps','*.png','*.svg'});
