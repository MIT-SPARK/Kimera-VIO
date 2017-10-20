fieldName = 'relTranErrors_vio'
%fieldName = 'overallVIO_times'
fieldName = 'tranErrors_vio_align'
condition = 4; % technique
nrDatasets = 11;
dataset = 8

for j=1:nrRuns
    for i = 1:nrDatasets
        ind = nrDatasets*(j-1) + i;
        subStruct = results(ind,condition).results;
        % vector corresponding to data over time of dataset i:
        temp = getfield(subStruct,fieldName);
        if length(temp)>1 % it is a vector
            temp = mean(temp);
        end
        datasetVSrun(i,j) = temp;
    end
end

thisDataset = datasetVSrun(dataset,:)

mean(thisDataset)
