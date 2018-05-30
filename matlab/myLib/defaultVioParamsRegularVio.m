function vioParams = defaultVioParams()

%% VIO PARAMS:
% IMU PARAMS
vioParams.gyroNoiseDensity = 0.00016968; % given by specs
vioParams.accNoiseDensity = 0.002;  % given by specs
vioParams.gyroBiasSigma = 1.9393e-05;
vioParams.accBiasSigma = 0.003;
vioParams.imuIntegrationSigma = 1.0e-08;
vioParams.n_gravity = [0.0, 0.0, -9.81];
vioParams.nominalImuRate = 0.005;
% INITIALIZATION PARAMS
vioParams.autoInitialize = 0;
vioParams.roundOnAutoInitialize = 0;
vioParams.initialPositionSigma = 0.00001;
vioParams.initialRollPitchSigma = 0.1745329519
vioParams.initialYawSigma = 0.00174532925;
vioParams.initialVelocitySigma = 0.001;
vioParams.initialAccBiasSigma = 0.1;
vioParams.initialGyroBiasSigma = 0.01;
% VISION PARAMS
vioParams.linearizationMode = 0;
vioParams.degeneracyMode = 1;
vioParams.smartNoiseSigma = 3;
vioParams.monoNoiseSigma = 0.85; % Only used for RegularVioBackEnd
vioParams.stereoNoiseSigma = 1.15; % Only used for RegularVioBackEnd
vioParams.regularityNoiseSigma = 0.05; % Only used for RegularVioBackEnd
vioParams.minPlaneConstraints = 14; % Only used for RegularVioBackEnd
vioParams.huberParam = 0.8 % Only used for RegularVioBackEnd
vioParams.tukeyParam = 4.2; % Only used for RegularVioBackEnd
vioParams.regularityNormType = 2; % Only used for RegularVioBackEnd. 0: square 1:huber 2:tukey
vioParams.rankTolerance = 1;
vioParams.landmarkDistanceThreshold = 20;
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
