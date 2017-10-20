%% RUN!
timeStart = fix(clock);
for testCond = 1:length(conditions)
    testCounter = 0;
    
    vioParams = defaultVioParams();
    trackerParams = defaultTrackerParams();
         
    conditionValue = conditions(testCond);
    if(trackerCondition)     
        trackerParams = setfield(trackerParams,conditionName,conditionValue);
    else
        vioParams = setfield(vioParams,conditionName,conditionValue);
    end
    % Make sure the feature age is compatible
    nrKfInHorizon = vioParams.horizon / trackerParams.intra_keyframe_time;
    trackerParams = setfield(trackerParams,'maxFeatureAge',ceil( nrKfInHorizon / 2 ));
    
    % Make sure that mono ransac threshold is ok:
    if(trackerParams.ransac_use_2point_mono)
        trackerParams = setfield(trackerParams,'ransac_threshold_mono',1e-6);
    else % 5-point method- we actually use same threshold
        trackerParams = setfield(trackerParams,'ransac_threshold_mono',1e-6);
    end
    
    % Make sure that stereo ransac threshold is ok:
    if(trackerParams.ransac_use_1point_stereo)
        trackerParams = setfield(trackerParams,'ransac_threshold_stereo',1);
    else % else 3-point method
        trackerParams = setfield(trackerParams,'ransac_threshold_stereo',0.3);
    end
    
    switch trackerParams.templ_cols
        case 11
            trackerParams = setfield(trackerParams,'templ_rows',3);
        case 41
            trackerParams = setfield(trackerParams,'templ_rows',5);
        case 71
            trackerParams = setfield(trackerParams,'templ_rows',7);
        case 101
            trackerParams = setfield(trackerParams,'templ_rows',11);
        otherwise
            error('incorrect choice of trackerParams.templ_cols')
    end
       
    for datasetToRun = allDatasetsToRun
        testCounter = testCounter + 1;
        fprintf('==========\n Testing condition: %d/%d, test %d/%d\n  ==========\n',...
            testCond,length(conditions),testCounter,length(allDatasetsToRun));
        
        [mean_rotErrors_vio_align, mean_tranErrors_vio_align, results(testCounter,testCond)] = ...
            regressionTests(trackerParams,vioParams,datasetToRun,testCond,testCounter);
        
        disp('rotation errors')
        rotationErrors(testCounter,testCond) = mean_rotErrors_vio_align;
        
        disp('rotation errors')
        translationErrors(testCounter,testCond) = mean_tranErrors_vio_align;
        
        disp('translation errors')
    end
    
    fix(clock) % display time
end
timeStart
timeEnd = fix(clock)
save(horzcat('result-summary-',conditionName));

% store results in suitable folder
folderName = horzcat('/media/luca/DATAPART1/resultsStereoVIO/result-',...
    num2str(timeEnd(1)),'-',num2str(timeEnd(2)),'-',num2str(timeEnd(3)),...
    '-',conditionName,'/')
mkdir(folderName)
myCommand = horzcat('sudo mv result-* ',folderName);
disp(myCommand)
system(myCommand)

%% plot results
colors = 'brgmckbrgmck';
legends = {};
figure
subplot(1,2,1); hold on
for i=1:size(rotationErrors,2)
    plot(rotationErrors(:,i),horzcat('-',colors(i)))
    legends{i} = horzcat('cond',num2str(i));
end
legend(legends)
xlabel('datasets'); ylabel('rot error (rad)')

legends = {};
subplot(1,2,2); hold on
for i=1:size(rotationErrors,2)
    plot(translationErrors(:,i),horzcat('-',colors(i)))
    legends{i} = horzcat('cond',num2str(i));
end
legend(legends)
xlabel('datasets'); ylabel('tran error (m)')