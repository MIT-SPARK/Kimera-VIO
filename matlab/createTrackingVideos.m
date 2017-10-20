% Create videos looking at the following folders:
% outputStereoTrackerImages-*
% outputTrackerImages-*
close all
clear all
clc

mainFolderPath = '/home/luca/Desktop/2016-08-25-resultsRegressionTests/';

% Get a list of all files and folders in this folder.
files = dir(mainFolderPath);

% Get a logical vector that tells which is a directory.
dirFlags = [files.isdir];

% Extract only those that are directories.
subFolders = files(dirFlags);

% Create a video for each folder:
fps = 20;
str1 = 'outputStereoTrackerImages';
str2 = 'outputTrackerImages';
for k = 1 : length(subFolders)
    folderName = subFolders(k).name;
    imagesInputPath = horzcat(mainFolderPath,'/',folderName,'/');
    videoOutputFilename = horzcat(mainFolderPath,'/',folderName,'-video');
	if strncmpi(str1,folderName,length(str1))
        createVideoFromImages(imagesInputPath,'StereoTrackerDisplayKeyframe_%d.png#',videoOutputFilename,fps)
    elseif strncmpi(str2,folderName,length(str2))
        createVideoFromImages(imagesInputPath,'trackerDisplayFrame_%d.png#',videoOutputFilename,fps)
    end
end

