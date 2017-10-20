% myclc
nrDatasetsToTest = 11;

myy1label ='estimation error';
myy2label ='nr. of DSP (in thousands)';
myxlabel = 'bit width'
maxy = 0.4;
y1lim = [0.05 maxy];

% case 1:
% bitwidth 32
% error 0.1764
% DSP 1030
% 
% case 2:
% bitwidth 21
% error 0.1955
% DSP 630
% 
% case 3:
% bitwidth 16
% error 0.1954
% DSP 607
% 
% case 4:
% bitwidth 12
% error 0.224
% DSP 509
% 
% We're using case 3.

%% Tradeoff plot
conditions = [32 21 16 12]
myfontsize = 16;
xticks = [1:length(conditions)];
errorStats = [0.1764 0.1955 0.1954 0.224];
timeStats = [1030 630 607 509] / 1000;
xlabels = {'32', '21', '16' ,'12'};

f = figure; set(gca,'fontsize',myfontsize); hold on
sz = 25; % size scatter
colors = 'ggyyrrgyrgyr';
markers = '^^ssoo^so^so';
setSaturated = [];
for k=1:nrDatasetsToTest
    c = colors(k); % depending on datasets
    m = markers(k);
    saturatedStats = min(bitwidthSweepResults(k,:),maxy); % saturates to maxy
    scatter(xticks, saturatedStats,c,m,'filled','MarkerEdgeColor',[0 0.5 0.5]);
    setSaturated = union(setSaturated, find(saturatedStats < bitwidthSweepResults(k,:)));
end
setSaturated = unique(setSaturated);
setSaturated = setSaturated(:)' % must be a row vector
% errorbar([1:length(conditions)],mean(errorStats),std(errorStats), 'linewidth' , 2)
% plot(mean(errorStats),'-b','linewidth' , 2)
% plot([1:length(conditions)],mean(errorStats)-std(errorStats),'--b','linewidth' , 2)
% plot([1:length(conditions)],mean(errorStats)+std(errorStats),'--b','linewidth' , 2)

[hAx,hLine1,hLine2] = plotyy(xticks,errorStats,xticks,timeStats);
if ~isempty(setSaturated)
    for ii = setSaturated
        arrowLen = 0.04;
        quiver(xticks(ii),maxy-arrowLen,0,arrowLen,'k','LineWidth',2,'ShowArrowHead','off')
        scatter(xticks(ii),maxy-0.1*arrowLen,'k','^','filled');
    end
end

set(hAx(1),'ylim',y1lim,'ytick',[0.1 0.2 0.3 0.4 0.5])
set(hAx(1),'fontsize',myfontsize);
set(hAx(2),'fontsize',myfontsize);

ylabel(hAx(1),myy1label) % left y-axis
ylabel(hAx(2),myy2label) % right y-axis
xlabel(myxlabel,'Interpreter','latex')

set(gca,'XTick',xticks)
set(gca,'XTickLabel',xlabels)

set(hAx,{'ycolor'},{'b';'r'});  % Left color red, right color blue...
hLine1.LineStyle = '-';
hLine1.LineWidth = 2;
hLine1.Color = 'b';

hLine2.LineStyle = '--';
hLine2.LineWidth = 2;
hLine2.Color = 'r';

filename = horzcat('error_vs_time-reducedPrecision');
print(f, filename,'-dpdf') % '-depsc')
system(horzcat('pdfcrop ',filename,'.pdf ',filename,'.pdf'));