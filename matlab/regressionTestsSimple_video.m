clear all
close all
clc

addpath('./myLib/')
%% CONDITIONS TO TEST
conditions = 4 % [1:6]; % 1=baseline, 2=quality, 3=eigmin, 4=logdet, 5=quality+binning, 6=baseline+binning
descriptionResults = horzcat('featureSelection',num2str(conditions))
datasetsToRun = [1:11]
nrDatasetsToTest = length(datasetsToRun);
nrRuns = 1;
allDatasetsToRun = repmat(datasetsToRun,1,nrRuns);

%% RUN!
useSudo = 1;
usePlain = 0;
vioParams = defaultVioParams();
trackerParams = defaultTrackerParams();
warning('AD HOC PARAMS')  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
trackerParams.min_distance = 4;
trackerParams.useSuccessProbabilities = 0;

vioParams.addBetweenStereoFactors = 0;
vioParams.relinearizeThreshold = 0;

nrFeaturesPerFrame = 100;
nrFeatSelect = 10;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
timeStart = fix(clock);
for testCond = conditions
    testCounter = 0;
    switch testCond  
        case 1 % baseline: large number of features
            trackerParams.maxFeaturesPerFrame = 200;
            trackerParams.featureSelectionNrCornersToSelect = trackerParams.maxFeaturesPerFrame;
            trackerParams.featureSelectionCriterion = 0; % quality
        case 2
            trackerParams.maxFeaturesPerFrame = nrFeaturesPerFrame;
            trackerParams.featureSelectionNrCornersToSelect = nrFeatSelect;
            trackerParams.featureSelectionCriterion = 0; % quality
        case 3
            trackerParams.maxFeaturesPerFrame = nrFeaturesPerFrame;
            trackerParams.featureSelectionNrCornersToSelect = nrFeatSelect;
            trackerParams.featureSelectionUseLazyEvaluation = 1;
            trackerParams.featureSelectionCriterion = 1; % eigmin
        case 4
            trackerParams.maxFeaturesPerFrame = nrFeaturesPerFrame;
            trackerParams.featureSelectionNrCornersToSelect = nrFeatSelect;
            trackerParams.featureSelectionUseLazyEvaluation = 0;
            trackerParams.featureSelectionCriterion = 2; % logdet
        case 5
            trackerParams.maxFeaturesPerFrame = nrFeaturesPerFrame;
            trackerParams.featureSelectionNrCornersToSelect = nrFeatSelect;
            trackerParams.featureSelectionCriterion = 0; % quality
            trackerParams.min_distance = 30; % binning
        case 6 % baseline + binning: expected to be best result
            trackerParams.maxFeaturesPerFrame = 200;
            trackerParams.featureSelectionNrCornersToSelect = trackerParams.maxFeaturesPerFrame;
            trackerParams.min_distance = 20; % binning (defult)
            trackerParams.featureSelectionCriterion = 0; % quality
        otherwise
            error('wrong choice of conditions')
    end
    
    for datasetToRun = allDatasetsToRun
        testCounter = testCounter + 1;
        fprintf('==========\n Testing condition: %d/%d, test %d/%d\n==========\n',...
            testCond,length(conditions),testCounter,length(allDatasetsToRun));
        
        [mean_rotErrors_vio_align, mean_tranErrors_vio_align, results(testCounter,testCond)] = ...
            regressionTests(trackerParams,vioParams,datasetToRun,testCond,testCounter,usePlain,useSudo);
        
        rotationErrors(testCounter,testCond) = mean_rotErrors_vio_align;
        translationErrors(testCounter,testCond) = mean_tranErrors_vio_align;
        
        timeFrontEnd(testCounter,testCond) = mean( results(testCounter,testCond).results.processStereoFrame_times );
        timeBackend(testCounter,testCond) = mean( results(testCounter,testCond).results.overallVIO_times ); 
        timeOverall(testCounter,testCond) = mean( results(testCounter,testCond).results.overall_times );
                
        timeLinearize(testCounter,testCond) = mean( results(testCounter,testCond).results.linearizeTime );
        timeLinearSolve(testCounter,testCond) = mean( results(testCounter,testCond).results.linearSolveTime );
        timeRetract(testCounter,testCond) = mean( results(testCounter,testCond).results.retractTime );
        timeLinearizeMarginals(testCounter,testCond) = mean( results(testCounter,testCond).results.linearizeMarginalizeTime );
        timeMarginalize(testCounter,testCond) = mean( results(testCounter,testCond).results.marginalizeTime );
        timeSum(testCounter,testCond) = timeLinearize(testCounter,testCond) + timeLinearSolve(testCounter,testCond) + ...
            timeRetract(testCounter,testCond)+ timeLinearizeMarginals(testCounter,testCond) + timeMarginalize(testCounter,testCond);
    end
    
    fix(clock) % display time
end
timeStart
timeEnd = fix(clock)
save(horzcat('result-summary-',descriptionResults));

%% plot final results
% PLOT_SELECTOR_RESULTS







