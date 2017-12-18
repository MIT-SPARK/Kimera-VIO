clear all
close all
clc

addpath('./myLib/')
%% CONDITIONS TO TEST
descriptionResults = 'gtsam_vs_plain'
conditions = [1 2]; % 1==original gtsam code, 2==plain cpp
allDatasetsToRun = [1:11 1:11 1:11];

%% RUN!
useSudo = 1;
fcn_file_path = mfilename('fullpath');
[pathstr, ~, ~] = fileparts(fcn_file_path);
if exist(fullfile(pathstr, 'mark_zhengdong'), 'file')
    system('source ~/.cshrc.zzd');
    useSudo = 0;
end

vioParams = defaultVioParams_sw();
trackerParams = defaultTrackerParams_sw();
        
timeStart = fix(clock);
for testCond = conditions
    testCounter = 0;
    switch testCond  
        case 1 % original gtsam code
            usePlain = 0;
        case 2 % code with tight params
            usePlain = 1;
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







