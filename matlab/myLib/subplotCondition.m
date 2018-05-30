function subplotCondition(r,c,i,paramName,runResults,conditions,nrRuns,field,fieldName)

subplot(r,c,i); hold on
xlabel(paramName); ylabel(fieldName,'interpreter','none');

for j = 1:nrRuns 
    v = zeros(length(conditions),1);
    for k = 1:length(conditions)
      v(k) = getfield(runResults(k,j).results,field); % put in vector form
    end
    plot(conditions,v,'-ob')
end

for j=1:length(conditions)
    labels{j} = num2str(round(conditions(j),2));
end

%set(gca,'XTick',1:length(conditions));
%set(gca,'XTickLabel',labels)
% set(gca,'YTicklabel',num2str(get(gca,'xtick')','%.2f'))