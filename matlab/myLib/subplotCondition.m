function subplotCondition(r,c,i,paramName,runResults,conditions,nrRuns,s)

subplot(r,c,i); hold on
xlabel(paramName); ylabel(s,'interpreter','none');
for j = 1:nrRuns 
    v = zeros(length(conditions),1);
    for i = 1:length(conditions)
      v(i) = getfield(runResults(i,j).results,s); % put in vector form
    end
    disp(v')
    plot(1:length(conditions),v,'-b')
end
for i=1:length(conditions)
    labels{i} = num2str(conditions(i));
end
set(gca,'XTick',1:length(conditions))
set(gca,'XTickLabel',labels)
% set(gca,'YTicklabel',num2str(get(gca,'xtick')','%.2f'))