function avePerDataset = averageRunsFromResultsStructPerDataset(results,fieldName,condition,nrDatasets,nrRuns,restrictToRuns)
% results has the following fields (we access only results):
%         results: [1x1 struct]
%                     vioParams: [1x1 struct]
%                 trackerParams: [1x1 struct]
%      mean_rotErrors_vio_align: 0.0338
%     mean_tranErrors_vio_align: 0.2019

if nargin < 6
    restrictToRuns = [1:nrRuns];
end

% create matrix of average results for each run and dataset
for j=1:nrRuns
    for i = 1:nrDatasets
        ind = nrDatasets*(j-1) + i;
        subStruct = results(ind,condition).results;
        % vector corresponding to data over time of dataset i:
        temp = getfield(subStruct,fieldName);
        if length(temp)>1 % it is a vector
            temp = mean(temp);
        end
        matAllDatasets(i,j) = temp;
    end
end

% datasets by rows, runs by columns
matAllDatasets = matAllDatasets(:,restrictToRuns);

% average results
% matAllDatasets
avePerDataset = mean(matAllDatasets,2);