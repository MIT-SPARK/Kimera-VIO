clear all
close all
clc

addpath('./myLib/')
%% CONDITIONS TO TEST
descriptionResults = 'sw_vs_hw_params_gtsam'
conditions = [2]; % 1==original code, 2==hw params
% allDatasetsToRun = [1:11] % [1:11 1:11 1:11];

allDatasetsToRun = [1, 2];

usePlain = 0;
useSudo = 0;

%% RUN!
fcn_file_path = mfilename('fullpath');
[pathstr, ~, ~] = fileparts(fcn_file_path);
if exist(fullfile(pathstr, 'mark_zhengdong'), 'file')
    system('source ~/.cshrc.zzd');
end

timeStart = fix(clock);
for testCond = conditions
    testCounter = 0;
    switch testCond       
        case 1 % original code
            vioParams = defaultVioParams();
            trackerParams = defaultTrackerParams();
        case 2 % code with tight params
            vioParams = defaultVioParams_sw();
            trackerParams = defaultTrackerParams_sw();
        otherwise
            error('wrong choice of conditions')
    end
    
    for datasetToRun = allDatasetsToRun
        testCounter = testCounter + 1;
        fprintf('==========\n Testing condition: %d/%d, test %d/%d\n  ==========\n',...
            testCond,length(conditions),testCounter,length(allDatasetsToRun));
        
        [mean_rotErrors_vio_align, mean_tranErrors_vio_align, results(testCounter,testCond)] = ...
            regressionTests(trackerParams,vioParams,datasetToRun,testCond,testCounter);
        
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

percentLinearize = 100 * timeLinearize ./ timeSum
percentLinearSolve = 100 * timeLinearSolve ./ timeSum
percentRetract = 100 * timeRetract ./ timeSum
percentMarginalize = 100 * (timeLinearizeMarginals + timeMarginalize) ./ timeSum

timeFrontEnd
timeBackend
timeOverall

%% Multi-threaded results (dataset 1)
% percentLinearize =
%    32.4166   33.7797
% percentLinearSolve =
%    58.4801   58.3762
% percentRetract =
%     0.6300    0.8005
% percentMarginalize =
%     8.4732    7.0435

%% single thread results (dataset 1)
% percentLinearize =
%    77.2608   81.5426
% percentLinearSolve =
%    17.9678   13.9331
% percentRetract =
%     0.2072    0.2041
% percentMarginalize =
%     4.5641    4.3202

% % store results in suitable folder
% folderName = horzcat('/media/luca/DATAPART1/resultsStereoVIO/result-',...
%     num2str(timeEnd(1)),'-',num2str(timeEnd(2)),'-',num2str(timeEnd(3)),...
%     '-',descriptionResults,'/')
% mkdir(folderName)
% myCommand = horzcat('sudo mv result-* ',folderName);
% disp(myCommand)
% system(myCommand)
% 
% % plot results
% figure
% subplot(1,2,1); hold on
% plot(rotationErrors(:,1),'-b')
% plot(rotationErrors(:,2),'-r')
% legend('sw','hw')
% 
% subplot(1,2,2); hold on
% plot(translationErrors(:,1),'-b')
% plot(translationErrors(:,2),'-r')
% legend('sw','hw')







