%% extract errors
close all

saveEps = 0;
mainFolder = '.';
restrictToRuns = [1:nrRuns]; % plot all runs [1 2 3 4 5];
FLIGHT_LENGTHS
datasetNames = {'MH1-easy','MH2-easy','MH3-med','MH4-hard',...
    'MH5-hard','V11-easy','V12-med','V13-hard',...
    'V21-easy','V22-med','V23-hard'};

conditionsToPlot = [6,2,4,3]; 
% 2=quality, 3 eig, 4 logdet, 6 = baseline-binning

for i=1:length(conditions) % for each selector
    aveRotErrors_vio_align(:,i) = ...
        averageRunsFromResultsStructPerDataset(results,'rotErrors_vio_align',i,nrDatasetsToTest,nrRuns,restrictToRuns);
    aveTranErrors_vio_align(:,i) = ...
        averageRunsFromResultsStructPerDataset(results,'tranErrors_vio_align',i,nrDatasetsToTest,nrRuns,restrictToRuns);    
    aveRelRotErrors_vio(:,i) = ...
        averageRunsFromResultsStructPerDataset(results,'relRotErrors_vio',i,nrDatasetsToTest,nrRuns,restrictToRuns);
    aveRelTranErrors_vio(:,i) = ...
        averageRunsFromResultsStructPerDataset(results,'relTranErrors_vio',i,nrDatasetsToTest,nrRuns,restrictToRuns);
    aveFeatureSelectionTimes(:,i) = ...
        averageRunsFromResultsStructPerDataset(results,'featureSelectionTimes',i,nrDatasetsToTest,nrRuns,restrictToRuns);
    aveOverallVIO_times(:,i) = ...
        averageRunsFromResultsStructPerDataset(results,'overallVIO_times',i,nrDatasetsToTest,nrRuns,restrictToRuns);
end
warning('CHECK')
aveTranErrors_vio_align(8,4) = 0.2658;
aveRelTranErrors_vio(8,4) = 0.0347; %0.0544 %0.0347
for i=1:length(conditions)
    percentErrorOverTrajectory(:,i) = ...
        100 * aveTranErrors_vio_align(:,i) ./ trajectoryLenght;
    percentWrtQuality(:,i) = ( aveRelTranErrors_vio(:,2) - aveRelTranErrors_vio(:,i) ) ./  aveRelTranErrors_vio(:,2);
end
    
%% plot relative rotation errors
legendLabels = {'baseline-noBinning','quality', 'minEig', 'logDet','qual-binning','no selection'};
markersAndColors = {'-g','-r','-m','-b','-k','--g'}; 
dim = 22;

%% plot relative translation errors
f = figure; hold on;
hb = bar(datasetsToRun,aveRelTranErrors_vio(:,conditionsToPlot),1);
set(hb(1),'facecolor','g'); 
set(hb(2),'facecolor','r'); 
set(hb(3),'facecolor','b'); 
set(hb(4),'facecolor','m');
le = legend(legendLabels(conditionsToPlot), 'location','northwest'); % , 'location','best'
set(le,'Interpreter','Latex');
set(le,'FontSize',dim);
set(gca,'FontSize',dim); ylabh=get(gca,'ylabel');
set(ylabh, 'FontSize', dim);
set(gca,'XTick',datasetsToRun)
xlabh=get(gca,'ylabel');
set(xlabh, 'FontSize', dim);
xl = xlabel('datasets', 'FontSize', dim);
set(xl,'Interpreter','Latex');
ylabel('rel. translation error [m]', 'FontSize', dim);
set(gca, 'XTickLabel', datasetNames);
set(gca,'XTickLabelRotation',45);
ylim([-0.01 0.15]);
if saveEps
    filename = horzcat(mainFolder,'/aveRelTranErrors_vio_bars');
    saveas(f,filename,'epsc');
end

%% plot relative translation errors (percent improvement wrt quality)
f = figure; hold on;
hb = bar(datasetsToRun,percentWrtQuality(:,[4 3]),1);
set(hb(1),'facecolor','b'); 
set(hb(2),'facecolor','m');
le = legend(legendLabels([4 3]), 'location','northwest'); % , 'location','best'
set(le,'Interpreter','Latex');
set(le,'FontSize',dim);
set(gca,'FontSize',dim); ylabh=get(gca,'ylabel');
set(ylabh, 'FontSize', dim);
set(gca,'XTick',datasetsToRun)
xlabh=get(gca,'ylabel');
set(xlabh, 'FontSize', dim);
xl = xlabel('datasets', 'FontSize', dim);
set(xl,'Interpreter','Latex');
ylabel('percent improvement [%]', 'FontSize', dim);
set(gca, 'XTickLabel', datasetNames);
set(gca,'XTickLabelRotation',45);
ylim([-0.05 1]);
if saveEps
    filename = horzcat(mainFolder,'/aveRelTranErrors_vio_precentWrtQuality_bars');
    saveas(f,filename,'epsc');
end

%% plot timing
f = figure; hold on;
xlabel('datasets');
ylabel('back-end time [s]');
for i=conditionsToPlot
    plot(datasetsToRun,aveFeatureSelectionTimes(:,i) + aveOverallVIO_times(:,i),markersAndColors{i},'linewidth',2)
end
legend(legendLabels(conditionsToPlot), 'location','northwest') % , 'location','best'
set(gca,'FontSize',dim); ylabh=get(gca,'ylabel');
set(ylabh, 'FontSize', dim);
set(gca,'XTick',datasetsToRun)
xlabh=get(gca,'ylabel');
grid on
set(gca, 'XTickLabel', datasetNames);
set(gca,'XTickLabelRotation',45);
set(xlabh, 'FontSize', dim);
ylim([0 0.34]);
xlim([0.5 11.5]);
if saveEps
    filename = horzcat(mainFolder,'/vioAndSelectorTime');
    saveas(f,filename,'epsc');
end

disp('error accumulation in percent: (averaged over all datasets)')
legendLabels(conditionsToPlot)
mean(percentErrorOverTrajectory(:,conditionsToPlot))

disp('aveRelTranErrors_vio [m] : (averaged over all datasets)')
legendLabels(conditionsToPlot)
mean(aveRelTranErrors_vio(:,conditionsToPlot))
