%% PLOT!!
myclc
conditionName = 'templ_cols'
myDate = '2017-1-29';
folderName = horzcat('/media/luca/DATAPART1/resultsStereoVIO/result-',myDate,...
    '-',conditionName,'/');
load(horzcat(folderName,'result-summary-',conditionName));
yData = 'processStereoFrame_times';
RUN_FIGUREMAKER

%% PLOT!!
myclc
conditionName = 'klt_max_level'
myDate = '2017-1-30';
folderName = horzcat('/media/luca/DATAPART1/resultsStereoVIO/result-',myDate,...
    '-',conditionName,'/');
load(horzcat(folderName,'result-summary-',conditionName));
yData = 'featureTrackingTimes';
RUN_FIGUREMAKER

%% PLOT!!
myclc
conditionName = 'horizon'
myDate = '2017-1-30';
folderName = horzcat('/media/luca/DATAPART1/resultsStereoVIO/result-',myDate,...
    '-',conditionName,'/');
load(horzcat(folderName,'result-summary-',conditionName));
yData = 'overallVIO_times';
RUN_FIGUREMAKER

%% PLOT!!
myclc
conditionName = 'numOptimize'
myDate = '2017-1-30';
folderName = horzcat('/media/luca/DATAPART1/resultsStereoVIO/result-',myDate,...
    '-',conditionName,'/');
load(horzcat(folderName,'result-summary-',conditionName));
yData = 'overallVIO_times';
RUN_FIGUREMAKER

%% PLOT!!
myclc
conditionName = 'maxFeaturesPerFrame'
myDate = '2017-1-27';
folderName = horzcat('/media/luca/DATAPART1/resultsStereoVIO/result-',myDate,...
    '-',conditionName,'/');
load(horzcat(folderName,'result-summary-',conditionName));
yData = 'overallVIO_times';
RUN_FIGUREMAKER

%% PLOT!!
myclc
conditionName = 'klt_win_size'
myDate = '2017-1-26';
folderName = horzcat('/media/luca/DATAPART1/resultsStereoVIO/result-',myDate,...
    '-',conditionName,'/');
load(horzcat(folderName,'result-summary-',conditionName));
yData = 'featureTrackingTimes';
RUN_FIGUREMAKER

%% PLOT!!
myclc
conditionName = 'intra_keyframe_time'
myDate = '2017-1-28';
folderName = horzcat('/media/luca/DATAPART1/resultsStereoVIO/result-',myDate,...
    '-',conditionName,'/');
load(horzcat(folderName,'result-summary-',conditionName));
yData = 'overall_times';
RUN_FIGUREMAKER

%% PLOT!!
myclc
conditionName = 'ransac_use_2point_mono'
myDate = '2017-1-27';
folderName = horzcat('/media/luca/DATAPART1/resultsStereoVIO/result-',myDate,...
    '-',conditionName,'/');
load(horzcat(folderName,'result-summary-',conditionName));
yData = 'monoRansacTimes';
RUN_FIGUREMAKER

%% PLOT!!
myclc
conditionName = 'ransac_use_1point_stereo'
myDate = '2017-1-27';
folderName = horzcat('/media/luca/DATAPART1/resultsStereoVIO/result-',myDate,...
    '-',conditionName,'/');
load(horzcat(folderName,'result-summary-',conditionName));
yData = 'stereoRansacTimes';
RUN_FIGUREMAKER






