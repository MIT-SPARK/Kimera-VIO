clear all
close all
clc

vioParams = defaultVioParams();
writeVioParamsYAML('./defaultVioParams.yaml', vioParams);

trackerParams = defaultTrackerParams();
writeTrackerParamsYAML('./defaultTrackerParams.yaml', trackerParams);