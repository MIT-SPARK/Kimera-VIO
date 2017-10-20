function [] = createDroneVideo(keyframeID,posesVIO,posesGT,resultsOutFolder,framerate)

if nargin<5
    framerate = 5;
end

%% plot trajectories
cameraSize = 0.2;
videoSkipStep = 10;
writerObj = VideoWriter(horzcat(resultsOutFolder,'outputTrajectory'),'Uncompressed AVI'); % 'mpeg-4'); %,'Uncompressed AVI'); %'Archival');
writerObj.FrameRate = framerate; % writerObj.Quality = 100;
open(writerObj);

% put translations in more readably form:
for i=1:length(keyframeID)
    tranVIO(:,i) = posesVIO(i).t; % by columns
    tranGT(:,i) = posesGT(i).t; % by columns
end

tmin = min([tranVIO tranGT]')';
tmax = max([tranVIO tranGT]')';
xmargin = 1;
ymargin = 1;
zmargin = 1;

fh = figure;
for i=1:videoSkipStep:length(keyframeID)
    clf
    axis equal
    
    xlim([tmin(1)-xmargin tmax(1)+xmargin])
    ylim([tmin(2)-ymargin tmax(2)+ymargin])
    zlim([tmin(3)-zmargin tmax(3)+zmargin])
    box on;  grid on
    view([-12 35])
    
    hold on;
    plot3(tranGT(1,1:i),tranGT(2,1:i),tranGT(3,1:i),'-g')
    plot3(tranVIO(1,1:i),tranVIO(2,1:i),tranVIO(3,1:i),'-b')
    myPlotCamera(posesVIO(i),0,cameraSize,'b');
    myPlotCamera(posesGT(i),0,cameraSize,'g');
    title(horzcat('GT (green) vs. VIO (blue) - Keyframe id: ',num2str(keyframeID(i))))
    drawnow
    
    set(fh, 'paperunits', 'points' )
    H = 800; W = H;
    set(fh,'position',[0, 0, W,H]); %
    set(fh,'papersize',[W,H]);
    try
        writeVideo(writerObj, getframe(fh));
    catch ME
        getReport(ME)
        warning('failed to write frame')
    end
end
close(writerObj);

% fh = figure; hold on;
% for i=1:length(keyframeID)
%    if rem(i,videoSkipStep)==1 % subsample
%        camVIO = plotCamera('Location',posesVIO(i).t,'Orientation',posesVIO(i).R,'Opacity',0,'Size',cameraSize);
%        if i==1
%            view(3)
%            axis equal
%            xmargin = 10;
%            ymargin = 10;
%            tstart = posesVIO(i).t;
%            % xlim([tstart(1)-xmargin tstart(1)+xmargin])
%            % ylim([tstart(2)-ymargin tstart(2)+ymargin])
%            % zlim([tstart(3)        tstart(3)+5])
%        end
%        if i>1
%            plotEdge3(posesVIO(i-1).t, posesVIO(i).t, '-r')
%        end
%        camGT = plotCamera('Location',posesGT(i).t,'Orientation',posesGT(i).R,'Opacity',0,'Size',cameraSize,'color','g');
%        if i>1
%            plotEdge3(posesGT(i-1).t, posesGT(i).t, '-g')
%        end
%        title(horzcat('GT (green) vs. VIO (red) - Keyframe id: ',num2str(keyframeID(i))))
%        drawnow
% 
%        set(fh, 'paperunits', 'points' )
%        H = 800; W = H;
%        set(fh,'position',[0, 0, W,H]); %
%        set(fh,'papersize',[W,H]);
%        try
%            writeVideo(writerObj, getframe(fh));
%        catch ME
%            warning('failed to write frame')
%        end
%    end
% end
% close(writerObj);