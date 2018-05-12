function writeVioParamsYAML(filename, vioParams)

% sanity check
nrFields = length(fieldnames(vioParams));
if(nrFields ~= 40)
   error('wrong number of fields in vioParams') 
end

fid = fopen(filename,'w');
fprintf(fid,'%%YAML:1.0\n');
fprintf(fid,'#VIO PARAMETERS\n');
% IMU PARAMS
fprintf(fid,'gyroNoiseDensity: %g\n', vioParams.gyroNoiseDensity);
fprintf(fid,'accNoiseDensity: %g\n', vioParams.accNoiseDensity);
fprintf(fid,'imuIntegrationSigma: %g\n', vioParams.imuIntegrationSigma);
fprintf(fid,'gyroBiasSigma: %g\n', vioParams.gyroBiasSigma);
fprintf(fid,'accBiasSigma: %g\n', vioParams.accBiasSigma);
fprintf(fid,'n_gravity: [%g, %g, %g]\n', vioParams.n_gravity(1),vioParams.n_gravity(2),vioParams.n_gravity(3));
fprintf(fid,'nominalImuRate: %g\n', vioParams.nominalImuRate);
% INITIALIZATION PARAMS
fprintf(fid,'autoInitialize: %d\n', vioParams.autoInitialize);
fprintf(fid,'roundOnAutoInitialize: %d\n', vioParams.roundOnAutoInitialize);
fprintf(fid,'initialPositionSigma: %10g\n', vioParams.initialPositionSigma);
fprintf(fid,'initialRollPitchSigma: %10g\n', vioParams.initialRollPitchSigma);
fprintf(fid,'initialYawSigma: %10g\n', vioParams.initialYawSigma);
fprintf(fid,'initialVelocitySigma: %10g\n', vioParams.initialVelocitySigma);
fprintf(fid,'initialAccBiasSigma: %10g\n', vioParams.initialAccBiasSigma);
fprintf(fid,'initialGyroBiasSigma: %10g\n', vioParams.initialGyroBiasSigma);
% VISION PARAMS
fprintf(fid,'linearizationMode: %d\n', vioParams.linearizationMode);
fprintf(fid,'degeneracyMode: %d\n', vioParams.degeneracyMode);
fprintf(fid,'smartNoiseSigma: %g\n', vioParams.smartNoiseSigma);
fprintf(fid,'monoNoiseSigma: %g\n', vioParams.monoNoiseSigma);
fprintf(fid,'regularityNoiseSigma: %g\n', vioParams.regularityNoiseSigma);
fprintf(fid,'minPlaneConstraints: %g\n', vioParams.minPlaneConstraints);
fprintf(fid,'huberParam: %g\n', vioParams.huberParam);
fprintf(fid,'rankTolerance: %g\n', vioParams.rankTolerance);
fprintf(fid,'landmarkDistanceThreshold: %g\n', vioParams.landmarkDistanceThreshold);
fprintf(fid,'outlierRejection: %g\n', vioParams.outlierRejection);
fprintf(fid,'retriangulationThreshold: %g\n', vioParams.retriangulationThreshold);
fprintf(fid,'addBetweenStereoFactors: %d\n', vioParams.addBetweenStereoFactors);
fprintf(fid,'betweenRotationPrecision: %g\n', vioParams.betweenRotationPrecision);
fprintf(fid,'betweenTranslationPrecision: %g\n', vioParams.betweenTranslationPrecision);
% OPTIMIZATION
fprintf(fid,'relinearizeThreshold: %g\n', vioParams.relinearizeThreshold);
fprintf(fid,'relinearizeSkip: %g\n', vioParams.relinearizeSkip);
fprintf(fid,'zeroVelocitySigma: %g\n', vioParams.zeroVelocitySigma);
fprintf(fid,'noMotionPositionSigma: %g\n', vioParams.noMotionPositionSigma);
fprintf(fid,'noMotionRotationSigma: %g\n', vioParams.noMotionRotationSigma);
fprintf(fid,'constantVelSigma: %g\n', vioParams.constantVelSigma);
fprintf(fid,'numOptimize: %d\n', vioParams.numOptimize);
fprintf(fid,'horizon: %g\n', vioParams.horizon);
fprintf(fid,'useDogLeg: %d\n', vioParams.useDogLeg);
fclose(fid);

