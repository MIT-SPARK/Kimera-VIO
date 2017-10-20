function mean_v = averageRuns(v,nrDatasets,nrRuns) 
% v is a vector containing nrRuns subvectors of nrDatasets elements.
% this function averages these subvectors and returns a single vector of nrDatasets elements.
    
v_matrix = reshape(v,nrDatasets,nrRuns);
mean_v = mean(v_matrix,2);