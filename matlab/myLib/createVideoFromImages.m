function createVideoFromImages(imagesInputPath,filePattern,videoOutputFilename,fps)
% addpath('./myLib')

if nargin < 1
    close all
    clc
    % filePattern = 'drawEpipolarLines_%d.png#';
    % imagesInputPath = './drawEpipolarLines/';
    
    % filePattern = 'rectifiedWithKeypointsAndDepth%d.png#';
    % imagesInputPath = './rectifiedWithKeypointsAndDepth/';
    
    fps = 20;
    filePattern = 'trackerDisplayFrame_%d.png#';
    imagesInputPath = '../build/outputImages/trackerDisplayFrame/';
    videoOutputFilename = './output';
end

writerObj = VideoWriter(videoOutputFilename,'Uncompressed AVI'); % 'mpeg-4'); %,'Uncompressed AVI'); %'Archival');
writerObj.FrameRate = fps; % writerObj.Quality = 100;
open(writerObj);

imagesInfo = dir(fullfile(imagesInputPath, '*.png'));

name = {imagesInfo.name};
str  = sprintf('%s#', name{:});
num  = sscanf(str, filePattern);
[~, index] = sort(num);
name = name(index);

for i=1:length(name)
    imageName = name{i}
    if length(imageName) < 10
        fprintf('skipped filename: %s\n', imageName)
        continue
    else
        fprintf('reading image: %s\n', imageName)
    end
    
    img = imread(horzcat(imagesInputPath,imageName));
    %imshow(img)
    writeVideo(writerObj,img);
end
close(writerObj);

% title('')
% axis tight
% xlabel('x [m]'); ylabel('y [m]')
% set(fh, 'paperunits', 'points' )
% set(fh,'papersize',[300 300]);
% set(fh,'PaperPositionMode','Auto')
% filename = horzcat(resultsFolder,'/frame');
% if (saveVideo) saveas(fh,filename,'pdf'); end
% axis tight
% axis equal
% set(fh, 'paperunits', 'points' )
% H = 800;
% W = H;
% set(fh,'position',[0, 0, W,H]); %
% set(fh,'papersize',[W,H]);
% % axis off
% writeVideo(writerObj, getframe(fh));