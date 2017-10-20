function writeTrackerParamsYAML(filename, trackerParams)

% sanity check
nrFields = length(fieldnames(trackerParams));
if(nrFields ~= 44)
   error('wrong number of fields in trackerParams') 
end

fid = fopen(filename,'w');
fprintf(fid,'%%YAML:1.0\n');
fprintf(fid,'#TRACKER PARAMETERS\n');
fprintf(fid,'klt_win_size: %d\n', trackerParams.klt_win_size);
fprintf(fid,'klt_max_iter: %d\n', trackerParams.klt_max_iter);
fprintf(fid,'klt_max_level: %d\n', trackerParams.klt_max_level);
fprintf(fid,'klt_eps: %g\n', trackerParams.klt_eps);
fprintf(fid,'maxFeatureAge: %d\n', trackerParams.maxFeatureAge);

fprintf(fid,'maxFeaturesPerFrame: %d\n', trackerParams.maxFeaturesPerFrame);
fprintf(fid,'quality_level: %g\n', trackerParams.quality_level);
fprintf(fid,'min_distance: %g\n', trackerParams.min_distance);
fprintf(fid,'block_size: %d\n', trackerParams.block_size);
fprintf(fid,'use_harris_detector: %d\n', trackerParams.use_harris_detector);
fprintf(fid,'k: %g\n', trackerParams.k);
fprintf(fid,'equalizeImage: %d\n', trackerParams.equalizeImage);

fprintf(fid,'nominalBaseline: %g\n', trackerParams.nominalBaseline);
fprintf(fid,'toleranceTemplateMatching: %g\n', trackerParams.toleranceTemplateMatching);
fprintf(fid,'templ_cols: %d\n', trackerParams.templ_cols);
fprintf(fid,'templ_rows: %d\n', trackerParams.templ_rows);
fprintf(fid,'stripe_extra_rows: %d\n', trackerParams.stripe_extra_rows);
fprintf(fid,'minPointDist: %g\n', trackerParams.minPointDist);
fprintf(fid,'maxPointDist: %g\n', trackerParams.maxPointDist);
fprintf(fid,'bidirectionalMatching: %d\n', trackerParams.bidirectionalMatching);
fprintf(fid,'subpixelRefinementStereo: %d\n', trackerParams.subpixelRefinementStereo);

fprintf(fid,'featureSelectionCriterion: %d\n', trackerParams.featureSelectionCriterion);
fprintf(fid,'featureSelectionHorizon: %g\n', trackerParams.featureSelectionHorizon);
fprintf(fid,'featureSelectionNrCornersToSelect: %d\n', trackerParams.featureSelectionNrCornersToSelect);
fprintf(fid,'featureSelectionImuRate: %g\n', trackerParams.featureSelectionImuRate);
fprintf(fid,'featureSelectionDefaultDepth: %g\n', trackerParams.featureSelectionDefaultDepth);
fprintf(fid,'featureSelectionCosineNeighborhood: %.15g\n', trackerParams.featureSelectionCosineNeighborhood);
fprintf(fid,'featureSelectionUseLazyEvaluation: %d\n', trackerParams.featureSelectionUseLazyEvaluation);
fprintf(fid,'useSuccessProbabilities: %d\n', trackerParams.useSuccessProbabilities);

fprintf(fid,'useRANSAC: %d\n', trackerParams.useRANSAC);
fprintf(fid,'minNrMonoInliers: %d\n', trackerParams.minNrMonoInliers);
fprintf(fid,'minNrStereoInliers: %d\n', trackerParams.minNrStereoInliers);
fprintf(fid,'ransac_threshold_mono: %g\n', trackerParams.ransac_threshold_mono);
fprintf(fid,'ransac_threshold_stereo: %g\n', trackerParams.ransac_threshold_stereo);
fprintf(fid,'ransac_use_1point_stereo: %d\n', trackerParams.ransac_use_1point_stereo);
fprintf(fid,'ransac_use_2point_mono: %d\n', trackerParams.ransac_use_2point_mono);
fprintf(fid,'ransac_max_iterations: %d\n', trackerParams.ransac_max_iterations);
fprintf(fid,'ransac_probability: %g\n', trackerParams.ransac_probability);
fprintf(fid,'ransac_randomize: %d\n', trackerParams.ransac_randomize);

fprintf(fid,'intra_keyframe_time: %g\n', trackerParams.intra_keyframe_time);
fprintf(fid,'minNumberFeatures: %d\n', trackerParams.minNumberFeatures);
fprintf(fid,'useStereoTracking: %d\n', trackerParams.useStereoTracking);

fprintf(fid,'display_time: %g\n', trackerParams.display_time);
fprintf(fid,'disparityThreshold: %g\n', trackerParams.disparityThreshold);

fclose(fid);
