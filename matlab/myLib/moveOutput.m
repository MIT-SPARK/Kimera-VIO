function moveOutput(folderName, extensionList)

disp('creating result folder:')
disp(folderName)

mkdir(folderName)

if nargin<2
    extensionList =  {'*.txt','*.eps','*.fig','*.avi'};
end

for i=1:length(extensionList)
    extension = extensionList{i};
    info = dir(fullfile('./', extension));
    names = {info.name};
    for i=1:length(names)
        name = names{i};
        movefile(horzcat('./',name),horzcat(folderName,'/',name))
    end
end




