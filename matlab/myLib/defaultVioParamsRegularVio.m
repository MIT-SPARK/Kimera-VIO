function vioParams = defaultVioParams()

%% VIO PARAMS:
% IMU PARAMS
vioParams.gyroNoiseDensity = 0.00016968; % given by specs
vioParams.accNoiseDensity = 0.002;  % given by specs
vioParams.gyroBiasSigma = 1.9393e-05;
vioParams.accBiasSigma = 0.003;
vioParams.imuIntegrationSigma = 1e-08;
vioParams.n_gravity = [0.0, 0.0, -9.81];
vioParams.nominalImuRate = 0.005;
% INITIALIZATION PARAMS
vioParams.autoInitialize = 0;
vioParams.roundOnAutoInitialize = 0;
vioParams.initialPositionSigma = 0.00001;
vioParams.initialRollPitchSigma = 10.0 / 180.0 * pi;
vioParams.initialYawSigma = 0.1 / 180.0 * pi;
vioParams.initialVelocitySigma = 1e-3;
vioParams.initialAccBiasSigma = 1e-1;
vioParams.initialGyroBiasSigma = 1e-2;
% VISION PARAMS
vioParams.linearizationMode = 0;
vioParams.degeneracyMode = 1;
vioParams.smartNoiseSigma = 2;
vioParams.monoNoiseSigma = 1.6; % Only used for RegularVioBackEnd
vioParams.stereoNoiseSigma = 1.6; % Only used for RegularVioBackEnd
vioParams.regularityNoiseSigma = 0.03; % Only used for RegularVioBackEnd
vioParams.minPlaneConstraints = 20; % Only used for RegularVioBackEnd
vioParams.huberParam = 0.8; % Only used for RegularVioBackEnd
vioParams.tukeyParam = 4.6851; % Only used for RegularVioBackEnd
vioParams.regularityNormType = 2; % Only used for RegularVioBackEnd. 0: square 1:huber 2:tukey
vioParams.rankTolerance = 1;
vioParams.landmarkDistanceThreshold = 15;
vioParams.outlierRejection = 8;
vioParams.retriangulationThreshold = 0.001;
vioParams.addBetweenStereoFactors = 1;
vioParams.betweenRotationPrecision = 0.0;
vioParams.betweenTranslationPrecision = 100;
% OPTIMIZATION
vioParams.relinearizeThreshold = 0.01;
vioParams.relinearizeSkip = 1;
vioParams.zeroVelocitySigma = 0.001;
vioParams.noMotionPositionSigma = 0.001;
vioParams.noMotionRotationSigma = 0.0001;
vioParams.constantVelSigma = 0.01;
vioParams.numOptimize = 0;
vioParams.horizon = 6;
vioParams.useDogLeg = 0;
