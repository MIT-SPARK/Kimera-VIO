function myPlotCamera(pose,opacity,cameraSize,color,refPose)

if nargin < 5
    refPose = eye(3);
end
%% compensate for the fact that the identity pose for the camera points up
% offsetRot = rotx(90); %  *offsetRot
plotCamera('Location',pose.t,'Orientation', pose.R' ,'Opacity',opacity,'Size',cameraSize,'color',color);

