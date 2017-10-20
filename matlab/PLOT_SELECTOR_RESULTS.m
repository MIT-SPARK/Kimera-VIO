%% extract errors
saveEps = 0;
mainFolder = '.';
FLIGHT_LENGTHS

conditionsToPlot = [1:length(conditions)]; % all

for i=1:length(conditions) % for each selector
    aveRotErrors_vio_align(:,i) = ...
        averageRunsFromResultsStructPerDataset(results,'rotErrors_vio_align',i,nrDatasetsToTest,nrRuns);
    aveTranErrors_vio_align(:,i) = ...
        averageRunsFromResultsStructPerDataset(results,'tranErrors_vio_align',i,nrDatasetsToTest,nrRuns);
    percentErrorOverTrajectory(:,i) = ...
        100 * aveTranErrors_vio_align(:,i) ./ trajectoryLenght;
    aveRelRotErrors_vio(:,i) = ...
        averageRunsFromResultsStructPerDataset(results,'relRotErrors_vio',i,nrDatasetsToTest,nrRuns);
    aveRelTranErrors_vio(:,i) = ...
        averageRunsFromResultsStructPerDataset(results,'relTranErrors_vio',i,nrDatasetsToTest,nrRuns);
    aveFeatureSelectionTimes(:,i) = ...
        averageRunsFromResultsStructPerDataset(results,'featureSelectionTimes',i,nrDatasetsToTest,nrRuns);
    aveOverallVIO_times(:,i) = ...
        averageRunsFromResultsStructPerDataset(results,'overallVIO_times',i,nrDatasetsToTest,nrRuns);
end
aveRotErrors_vio_align
aveTranErrors_vio_align

aveRelRotErrors_vio
aveRelTranErrors_vio

aveFeatureSelectionTimes
aveOverallVIO_times

%% plot relative rotation errors
legendLabels = {'baseline','quality', 'minEig', 'logdet','qual-binning','base-binning'};
markersAndColors = {'-g','-r','-m','-b','-k','--g'}; 
dim = 22;

f = figure; hold on;
xlabel('datasets');
ylabel('rel. rotation error [m]');
for i = conditionsToPlot
    plot(datasetsToRun,aveRelRotErrors_vio(:,i),markersAndColors{i},'linewidth',2)
end
legend(legendLabels(conditionsToPlot)) % , 'location','best'
set(gca,'FontSize',dim); ylabh=get(gca,'ylabel');
set(ylabh, 'FontSize', dim);
set(gca,'XTick',datasetsToRun)
xlabh=get(gca,'ylabel');
set(xlabh, 'FontSize', dim);
if saveEps
    filename = horzcat(mainFolder,'/aveRelRotErrors_vio');
    saveas(f,filename,'epsc');
end

%% plot relative translation errors
f = figure; hold on;
xlabel('datasets');
ylabel('rel. translation error [m]');
for i=conditionsToPlot
    plot(datasetsToRun,aveRelTranErrors_vio(:,i),markersAndColors{i},'linewidth',2)
end
legend(legendLabels(conditionsToPlot)) % , 'location','best'
set(gca,'FontSize',dim); ylabh=get(gca,'ylabel');
set(ylabh, 'FontSize', dim);
set(gca,'XTick',datasetsToRun)
xlabh=get(gca,'ylabel');
set(xlabh, 'FontSize', dim);
if saveEps
    filename = horzcat(mainFolder,'/aveRelTranErrors_vio');
    saveas(f,filename,'epsc');
end

%% plot relative translation errors (percent wrt quality)
f = figure; hold on;
xlabel('datasets');
ylabel('percent wrt quality [m]');
for i=conditionsToPlot
    percentWrtQuality = ( aveRelTranErrors_vio(:,2) - aveRelTranErrors_vio(:,i) ) ./  aveRelTranErrors_vio(:,2);
    plot(datasetsToRun,percentWrtQuality,markersAndColors{i},'linewidth',2)
end
legend(legendLabels(conditionsToPlot)) % , 'location','best'
set(gca,'FontSize',dim); ylabh=get(gca,'ylabel');
set(ylabh, 'FontSize', dim);
set(gca,'XTick',datasetsToRun)
xlabh=get(gca,'ylabel');
set(xlabh, 'FontSize', dim);
if saveEps
    filename = horzcat(mainFolder,'/percentWrtQuality');
    saveas(f,filename,'epsc');
end

%% plot relative aveTranErrors_vio_align
f = figure; hold on;
xlabel('datasets');
ylabel('ave Tran Errors align [m]');
for i=conditionsToPlot
    plot(datasetsToRun,aveTranErrors_vio_align(:,i),markersAndColors{i},'linewidth',2)
end
legend(legendLabels(conditionsToPlot)) % , 'location','best'
set(gca,'FontSize',dim); ylabh=get(gca,'ylabel');
set(ylabh, 'FontSize', dim);
set(gca,'XTick',datasetsToRun)
xlabh=get(gca,'ylabel');
set(xlabh, 'FontSize', dim);
if saveEps
    filename = horzcat(mainFolder,'/aveTranErrors_vio_align');
    saveas(f,filename,'epsc');
end

%% plot relative aveTranErrors_vio_align (% of trajectory)
f = figure; hold on;
xlabel('datasets');
ylabel('errors over distance travelled [m]');
for i=conditionsToPlot
    plot(datasetsToRun,percentErrorOverTrajectory(:,i),markersAndColors{i},'linewidth',2)
end
legend(legendLabels(conditionsToPlot)) % , 'location','best'
set(gca,'FontSize',dim); ylabh=get(gca,'ylabel');
set(ylabh, 'FontSize', dim);
set(gca,'XTick',datasetsToRun)
xlabh=get(gca,'ylabel');
set(xlabh, 'FontSize', dim);
if saveEps
    filename = horzcat(mainFolder,'/aveTranErrors_vio_align_percent_traj');
    saveas(f,filename,'epsc');
end



%% plot timing
f = figure; hold on;
xlabel('datasets');
ylabel('vio+selector time [s]');
for i=conditionsToPlot
    plot(datasetsToRun,aveFeatureSelectionTimes(:,i) + aveOverallVIO_times(:,i),markersAndColors{i},'linewidth',2)
end
legend(legendLabels(conditionsToPlot)) % , 'location','best'
set(gca,'FontSize',dim); ylabh=get(gca,'ylabel');
set(ylabh, 'FontSize', dim);
set(gca,'XTick',datasetsToRun)
xlabh=get(gca,'ylabel');
set(xlabh, 'FontSize', dim);
if saveEps
    filename = horzcat(mainFolder,'/vioAndSelectorTime');
    saveas(f,filename,'epsc');
end

%% plot timing breakdown
f = figure; hold on;
xlabel('datasets');
ylabel('vio+selector time [s]');
plot(datasetsToRun,aveFeatureSelectionTimes(:,3),'-m','linewidth',2)
plot(datasetsToRun,aveOverallVIO_times(:,3),':m','linewidth',2)
plot(datasetsToRun,aveFeatureSelectionTimes(:,4),'-b','linewidth',2)
plot(datasetsToRun,aveOverallVIO_times(:,4),':b','linewidth',2)

legend('eig-sel','eig-opt','det-sel','det-opt') % , 'location','best'
set(gca,'FontSize',dim); ylabh=get(gca,'ylabel');
set(ylabh, 'FontSize', dim);
set(gca,'XTick',datasetsToRun)
xlabh=get(gca,'ylabel');
set(xlabh, 'FontSize', dim);
if saveEps
    filename = horzcat(mainFolder,'/timingBreakdown');
    saveas(f,filename,'epsc');
end
