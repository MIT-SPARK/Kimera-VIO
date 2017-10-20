function v_mean = averageRunsFromResultsStruct(results,fieldName,condition,nrDatasets,nrRuns)
% results has the following fields (we access only results):
%         results: [1x1 struct]
%                     vioParams: [1x1 struct]
%                 trackerParams: [1x1 struct]
%      mean_rotErrors_vio_align: 0.0338
%     mean_tranErrors_vio_align: 0.2019

% create long vector
v = zeros(nrDatasets * nrRuns,1);
for i =1:nrDatasets * nrRuns
    subStruct = results(i,condition).results;
    temp = getfield(subStruct,fieldName);
    if length(temp)>1 % it is a vector
       temp = mean(temp); 
    end
    v(i) = temp;
end


% average results
v_mean = averageRuns(v,nrDatasets,nrRuns);