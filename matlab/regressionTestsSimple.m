clear all
close all
clc

addpath('./myLib/')
%% CONDITIONS TO TEST
runRegularVio = false;
if (runRegularVio)
    descriptionResults = 'regularVioTest'
else
    descriptionResults = 'VioTest'
end
conditions = [1];
datasetsToRun = [1:11]
% datasetsToRun = 6
nrDatasetsToTest = length(datasetsToRun);
nrRuns = 1;
allDatasetsToRun = repmat(datasetsToRun,1,nrRuns);
% TODO remove usePlain from everywhere...
usePlain = 0;
%% RUN!
useSudo = 0;
if (runRegularVio)
    vioParams = defaultVioParamsRegularVio();
    trackerParams = defaultTrackerParamsRegularVio();
else
    vioParams = defaultVioParams();
    trackerParams = defaultTrackerParams();
end
warning('AD HOC PARAMS')  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% TODO change params to the ones in pipeline

%nrFeaturesPerFrame = 100;
%nrFeatSelect = 100; % Avoid feature selection by setting it equal to above
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
timeStart = fix(clock);
for testCond = conditions
    testCounter = 0;
    switch testCond  
        case 1 % baseline: large number of features
            % TODO use to test different conditions... like diff params.
        otherwise
            error('wrong choice of conditions')
    end
    
    for datasetToRun = allDatasetsToRun
        testCounter = testCounter + 1;
        fprintf('==========\n Testing condition: %d/%d, test %d/%d\n==========\n',...
            testCond,length(conditions),testCounter,length(allDatasetsToRun));
        
        [mean_rotErrors_vio_align, mean_tranErrors_vio_align, results(testCounter,testCond)] = ...
            regressionTests(trackerParams,vioParams,datasetToRun,testCond,testCounter,usePlain,useSudo,runRegularVio);

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
%PLOT_SELECTOR_RESULTS







