%% axis labels:
myy1label ='estimation error';
myy2label ='time';
myxlabel = 'parameter'
maxy = 0.4;
y1lim = [0.05 maxy];
switch yData
    case 'processStereoFrame_times'
        myy2label = 'stereo matching time'
    case 'featureTrackingTimes'
        myy2label = 'feature tracking time'
    case 'overallVIO_times'
        myy2label = 'back-end time'
    case 'overall_times'
        myy2label = 'total time'
    case 'monoRansacTimes'
        myy2label = 'mono RANSAC time'
    case 'stereoRansacTimes'
        myy2label = 'stereo RANSAC time'
    otherwise error('wrong yData')
end

switch conditionName
    case 'klt_win_size'
        myxlabel = 'tracking window size ($T_{size}$)'        
    case 'templ_cols'
        myxlabel = 'template size ($S_{rows} \times S_{cols}$)'
    case 'klt_max_level'
        myxlabel = 'max. tracking level ($T_{levels}$)'
    case 'horizon'
        myxlabel = 'horizon ($h$, seconds)'
    case 'numOptimize'
        myxlabel = 'max. optimization iter. ($N_{iters}$)'
    case 'maxFeaturesPerFrame'
        myxlabel = 'max. detected features ($N_{f}$)'
    case 'intra_keyframe_time'
        myxlabel = 'intra-keyframe time ($\Delta t_{ij}$, seconds)'
    case 'ransac_use_2point_mono'
        myxlabel = 'mono RANSAC algorithm'
    case 'ransac_use_1point_stereo' 
        myxlabel = 'stereo RANSAC algorithm'
    otherwise error('wrong conditionName')
end

%% plot results
colors = 'brgmckbrgmck';
legends = {};
figure
subplot(1,2,1); hold on
for i=1:size(rotationErrors,2)
    plot(rotationErrors(:,i),horzcat('-',colors(i)))
    legends{i} = horzcat('cond',num2str(i));
end
legend(legends)
xlabel('datasets'); ylabel('rot error (rad)')

legends = {};
subplot(1,2,2); hold on
for i=1:size(rotationErrors,2)
    plot(translationErrors(:,i),horzcat('-',colors(i)))
    legends{i} = horzcat('cond',num2str(i));
end
legend(legends)
xlabel('datasets'); ylabel('tran error (m)')

%% Tradeoff plot
myfontsize = 18;
errorStats = zeros(nrDatasetsToTest,length(conditions)); 
xticks = [1:length(conditions)];
for i=1:length(conditions) % for each parameter choice
  errorStats(:,i) = averageRuns(translationErrors(:,i),nrDatasetsToTest,nrRuns);
  timeStats(:,i) = averageRunsFromResultsStruct(results,yData,i,nrDatasetsToTest,nrRuns);
  % overall_times featureTrackingTimes, monoRansacTimes. stereoRansacTimes
  xlabels{i} = num2str(conditions(i));
end
f = figure; set(gca,'fontsize',myfontsize); hold on
sz = 25; % size scatter
colors = 'ggyyrrgyrgyr';
markers = '^^ssoo^so^so';
setSaturated = [];
for k=1:nrDatasetsToTest
    c = colors(k); % depending on datasets
    m = markers(k);
    saturatedStats = min(errorStats(k,:),maxy); % saturates to maxy
    scatter(xticks, saturatedStats,c,m,'filled','MarkerEdgeColor',[0 0.5 0.5]);
    setSaturated = union(setSaturated, find(saturatedStats < errorStats(k,:)));
end
setSaturated = unique(setSaturated);
setSaturated = setSaturated(:)' % must be a row vector
% errorbar([1:length(conditions)],mean(errorStats),std(errorStats), 'linewidth' , 2)
% plot(mean(errorStats),'-b','linewidth' , 2)
% plot([1:length(conditions)],mean(errorStats)-std(errorStats),'--b','linewidth' , 2)
% plot([1:length(conditions)],mean(errorStats)+std(errorStats),'--b','linewidth' , 2)
switch conditionName
    case 'templ_cols'
        xlabels = {'11x3','41x5','71x7','101x11'}
    case 'klt_max_level'
        xlabels = {'1','2','3','4','5'}
    case 'numOptimize'
        xlabels = {'1','3','5'}
end

[hAx,hLine1,hLine2] = plotyy(xticks,mean(errorStats),xticks,mean(timeStats));
if ~isempty(setSaturated)
    for ii = setSaturated
        arrowLen = 0.04;
        quiver(xticks(ii),maxy-arrowLen,0,arrowLen,'k','LineWidth',2,'ShowArrowHead','off')
        scatter(xticks(ii),maxy-0.1*arrowLen,'k','^','filled');
    end
end
set(gca,'XTick',xticks)
set(gca,'XTickLabel',xlabels)
xlabel(myxlabel,'Interpreter','latex')

set(hAx,{'ycolor'},{'b';'r'});  % Left color red, right color blue...
hLine1.LineStyle = '-';
hLine1.LineWidth = 2;
hLine1.Color = 'b';

hLine2.LineStyle = '--';
hLine2.LineWidth = 2;
hLine2.Color = 'r';

set(hAx(1),'ylim',y1lim,'ytick',[0.1 0.2 0.3 0.4 0.5])
set(hAx,'fontsize',myfontsize);

% axis tight
% marginx = 0.2;
% xlimOld = xlim;
% xlim([xlimOld(1)-marginx xlimOld(2)+marginx])
% ylimOld = ylim;
% marginy = abs(ylimOld(2) - ylimOld(1))/10;
% ylim([ylimOld(1)-marginy ylimOld(2)+marginy])

ylabel(hAx(1),myy1label) % left y-axis
ylabel(hAx(2),myy2label) % right y-axis
filename = horzcat('error_vs_time-',conditionName);
print(f, filename,'-dpdf') % '-depsc')
system(horzcat('pdfcrop ',filename,'.pdf ',filename,'.pdf'));