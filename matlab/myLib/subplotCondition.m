function subplotCondition(r,c,i,runResults,datasetToRun,nrRuns,s)

subplot(r,c,i); hold on
xlabel('datasets'); ylabel(s,'interpreter','none');
for j = 1:nrRuns 
    v = zeros(length(datasetToRun),1);
    for i = 1:length(datasetToRun)
      v(i) = getfield(runResults(i,j).results,s); % put in vector form
    end
    disp(v')
    plot(1:length(datasetToRun),v,'-b')
end
for i=1:length(datasetToRun)
    labels{i} = num2str(datasetToRun(i));
end
set(gca,'XTick',datasetToRun)
set(gca,'XTickLabel',labels)
% set(gca,'YTicklabel',num2str(get(gca,'xtick')','%.2f'))