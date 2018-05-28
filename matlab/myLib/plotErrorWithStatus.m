function f = plotErrorWithStatus(rotErrors,tranErrors,testStatus,name)
% plotErrorWithStatus:
% plot rotations and translations errors, showing colors for each data
% points, associated with testStatus (0 = good status), (1 = bad status)

f = figure;
set(f, 'Visible', 'off');
testStatus(1) = 0;
testStatus(end) = 1;

n = length(rotErrors);
if length(tranErrors)~=n || length(testStatus)~=n
    error('plotErrorWithStatus: inconsistent length of error vectors')
end
myColor = jet;
middleCol = round(size(myColor,1)/2);
myColor = myColor(middleCol:end,:); % from green to red

subplot(1,2,1); hold on
colormap(myColor);
plot(rotErrors,'-k','linewidth',2); hold on
scatter([1:n],rotErrors,50,testStatus,'filled') % linspace(0,1,n)
xlabel('keyframe')
ylabel(horzcat(name,' rotation errors'))

subplot(1,2,2); hold on
colormap(myColor);
plot(tranErrors,'-k','linewidth',2); hold on
scatter([1:n],tranErrors,50,testStatus,'filled')
xlabel('keyframe')
ylabel(horzcat(name,' translation errors'))
set(f,'name','VALID (greed), LOW-DISPARITY, FEW-MATCHES, INVALID (red)','numbertitle','off')