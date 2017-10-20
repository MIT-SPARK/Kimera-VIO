function [fh] = plotDroneTrajectories(tranGT,tranVIO)

tmin = min([tranVIO; tranGT])';
tmax = max([tranVIO; tranGT])';
xmargin = 1;
ymargin = 1;
zmargin = 1;

fh = figure;
axis equal
box on;  grid on
view([-12 35])
hold on;
plot3(tranGT(:,1),tranGT(:,2),tranGT(:,3),'-g')
plot3(tranVIO(:,1),tranVIO(:,2),tranVIO(:,3),'-b')
xlim([tmin(1)-xmargin tmax(1)+xmargin])
ylim([tmin(2)-ymargin tmax(2)+ymargin])
zlim([tmin(3)-zmargin tmax(3)+zmargin])
title(horzcat('GT (green) vs. VIO (blue) - after alignment'))
drawnow

set(fh, 'paperunits', 'points' )
H = 800; W = H;
set(fh,'position',[0, 0, W,H]); %
set(fh,'papersize',[W,H]);



