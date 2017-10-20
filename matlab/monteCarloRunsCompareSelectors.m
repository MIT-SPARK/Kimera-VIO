clear all
close all
clc

%% run monte carlo
addpath('./myLib')
nrTests = 20;
maxCond = 5;
mainFolder = './monteCarloResults';
mkdir(mainFolder);
for test=1:nrTests
    seed = test;
    for cond = 1:maxCond
        tictimeMat = tic;
        switch(cond)
            case 1
                criterion = 0; % QUALITY
                lazy = 1; % does not really matter
            case 2
                criterion = 1; % MIN_EIG
                lazy = 1; 
            case 3
                criterion = 2; % LOGDET
                lazy = 1; 
            case 4
                criterion = 1; % MIN_EIG
                lazy = 0; 
            case 5
                criterion = 2; % LOGDET
                lazy = 0; 
            otherwise
                error('wrong cond')
        end
        fprintf('Test %d/%d, criterion %d/%d\n',test,nrTests,cond,maxCond)
        removeOutput(); pause(5)
        myCommand = horzcat('sudo ../build/stereoVIOExampleSimulation ',num2str(criterion),...
            ' ',num2str(seed),' ',num2str(lazy), ' > out.txt');
        disp(myCommand); system(myCommand)
        % run
        runResults(test,cond).results = visualizeResultsVIO('./');
        folderName = horzcat(mainFolder,'/monteCarloResult-',num2str(criterion),'_',num2str(seed),'_',num2str(lazy));
        moveOutput(folderName)
        pause(5)
        timeMat(test,cond) = toc(tictimeMat)
    end
end
save(horzcat('monteCarloResult_nrTests',num2str(nrTests)))

%% extract errors
n = length(runResults(1,1).results.tranErrors_vio); % length of a regular test
succTest = 0;
for test=1:nrTests
    for cond = 1:maxCond
        if length(runResults(test,cond).results.tranErrors_vio) == n
            succTest = succTest+1;
            errorResults(cond).vioTranErrorMatrix(succTest,:) = ...
                runResults(test,cond).results.tranErrors_vio;
            errorResults(cond).vioRotErrorMatrix(succTest,:) = ...
                runResults(test,cond).results.rotErrors_vio;
            selectionTiming(cond).matrix(succTest,:) = ...
                runResults(test,cond).results.featureSelectionTimes;
            errorResultsVioRelTranErrorMatrix(succTest,cond) = ...
                runResults(test,cond).results.meanRelTranErrors_vio;
            errorResultsVioRelRotErrorMatrix(succTest,cond) = ...
                runResults(test,cond).results.meanRelRotErrors_vio;
        else
            warning('failed test %d, condition %d',test, cond )
        end
    end
end

%% plot translation errors
dim = 22;
f = figure; hold on;
xlabel('keyframes');
ylabel('abs. translation error [m]');
% mean is by columns
plot(mean(errorResults(1).vioTranErrorMatrix),'--k','linewidth',2)
plot(mean(errorResults(2).vioTranErrorMatrix),':r','linewidth',2)
plot(mean(errorResults(3).vioTranErrorMatrix),'-b','linewidth',2)
%plot(mean(errorResults(4).vioTranErrorMatrix),':y','linewidth',2) %identical to lazy
%plot(mean(errorResults(5).vioTranErrorMatrix),':c','linewidth',2) %identical to lazy
legend('random', 'minEig', 'logdet') % , 'location','best'
set(gca,'FontSize',dim); ylabh=get(gca,'ylabel');
set(ylabh, 'FontSize', dim);
xlabh=get(gca,'ylabel');
set(xlabh, 'FontSize', dim);
filename = horzcat(mainFolder,'/results_monteCarlo_tranErrors');
saveas(f,filename,'epsc');

%% plot rotation errors
f = figure; hold on;
xlabel('keyframes');
ylabel('abs. rotation error [rad]');
% mean is by columns
plot(mean(errorResults(1).vioRotErrorMatrix),'--k','linewidth',2)
plot(mean(errorResults(2).vioRotErrorMatrix),':r','linewidth',2)
plot(mean(errorResults(3).vioRotErrorMatrix),'-b','linewidth',2)
%plot(mean(errorResults(4).vioRotErrorMatrix),':y','linewidth',2) %identical to lazy
%plot(mean(errorResults(5).vioRotErrorMatrix),':c','linewidth',2) %identical to lazy
legend('random', 'minEig', 'logdet') % , 'location','best'
set(gca,'FontSize',dim); ylabh=get(gca,'ylabel');
set(ylabh, 'FontSize', dim);
xlabh=get(gca,'ylabel');
set(xlabh, 'FontSize', dim);
filename = horzcat(mainFolder,'/results_monteCarlo_rotErrors');
saveas(f,filename,'epsc');

%% feature selection time
f = figure; hold on;
xlabel('keyframes');
ylabel('feature selection time [s]');
% mean is by columns
% plot(mean(selectionTiming(1).matrix),':r','linewidth',2)
semilogy(log10( mean(selectionTiming(2).matrix) ),'-k','linewidth',2)
semilogy(log10( mean(selectionTiming(3).matrix) ),'-b','linewidth',2)
semilogy(log10( mean(selectionTiming(4).matrix) ),':k','linewidth',2) 
semilogy(log10( mean(selectionTiming(5).matrix) ),':b','linewidth',2)
legend('minEig', 'logdet','minEig-nonLazy', 'logdet-nonLazy') % , 'location','best'
set(gca,'FontSize',dim); ylabh=get(gca,'ylabel');
set(ylabh, 'FontSize', dim);
xlabh=get(gca,'ylabel');
set(xlabh, 'FontSize', dim);
filename = horzcat(mainFolder,'/results_monteCarlo_timing');
saveas(f,filename,'epsc');
mean( mean(selectionTiming(2).matrix) )
mean( mean(selectionTiming(3).matrix) )
mean( mean(selectionTiming(4).matrix) ) 
mean( mean(selectionTiming(5).matrix) )

mean(errorResultsVioRelTranErrorMatrix)
mean(errorResultsVioRelRotErrorMatrix)