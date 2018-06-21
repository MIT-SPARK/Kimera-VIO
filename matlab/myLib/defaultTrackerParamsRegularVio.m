function trackerParams = defaultTrackerParams()

%% TRACKER PARAMS:
trackerParams.klt_win_size = 24;
trackerParams.klt_max_iter = 30;
trackerParams.klt_max_level = 4;
trackerParams.klt_eps = 0.1;
trackerParams.maxFeatureAge = 25;
%
trackerParams.maxFeaturesPerFrame = 600;
trackerParams.quality_level = 0.001;
trackerParams.min_distance = 20.0;
trackerParams.block_size = 3;
trackerParams.use_harris_detector = 0;
trackerParams.k = 0.04;
trackerParams.equalizeImage = 0;
%
trackerParams.nominalBaseline = 0.11;
trackerParams.toleranceTemplateMatching = 0.15;
trackerParams.templ_cols = 101;
trackerParams.templ_rows = 11;
trackerParams.stripe_extra_rows = 0;
trackerParams.minPointDist = 0.5;
trackerParams.maxPointDist = 10;
trackerParams.bidirectionalMatching = 0;
trackerParams.subpixelRefinementStereo = 0;
%
trackerParams.featureSelectionCriterion = 0;
trackerParams.featureSelectionHorizon = 3;
trackerParams.featureSelectionNrCornersToSelect = 600;
trackerParams.featureSelectionImuRate = 0.005;
trackerParams.featureSelectionDefaultDepth = 2.0;
trackerParams.featureSelectionCosineNeighborhood = cos( (10*pi)/(180.0) );
trackerParams.featureSelectionUseLazyEvaluation = 1;
trackerParams.useSuccessProbabilities = 1;
%
trackerParams.useRANSAC = 1;
trackerParams.minNrMonoInliers = 10;
trackerParams.minNrStereoInliers = 5;
trackerParams.ransac_threshold_mono = 1e-6;
trackerParams.ransac_threshold_stereo = 1;
trackerParams.ransac_use_1point_stereo = 1;
trackerParams.ransac_use_2point_mono = 1;
trackerParams.ransac_max_iterations = 100;
trackerParams.ransac_probability = 0.995;
trackerParams.ransac_randomize = 0;
%
trackerParams.intra_keyframe_time = 0.2;
trackerParams.minNumberFeatures = 0;
trackerParams.useStereoTracking = true;
%
trackerParams.display_time = 100;
trackerParams.disparityThreshold = 0.5;