disp('running script:')

appName = 'build/stereoVIOEuroc';

if exist('useSudo', 'var') && useSudo == 0
    sudoName = '';
else
    sudoName = 'sudo ';
end

% setenv('LD_LIBRARY_PATH', '/opt/intel/parallel_studio_xe_2018/compilers_and_libraries_2018/linux/mkl/bin/mklvars.sh intel64')
% if initialFrameID ==-1 || finalFrameID == -1
%     % use default initial and final frame
%     % TODO change command to use gflags
%     myCommand = horzcat('%s../%s ',datasetPath,' ',filenameVioParams,' ',...
%         filenameTrackerParams,' > out-',num2str(i),'.txt');
% else
%  myCommand = horzcat('%s../%s ',datasetPath,' ',filenameVioParams,' ',...
%         filenameTrackerParams,' ',num2str(initialFrameID),' ',num2str(finalFrameID),...
%         ' > out-',num2str(i),'.txt');
% end

backend_type = 0; % backend_type 0 for Normal VIO, 1 for Regular VIO.
myCommand = horzcat('%s../%s ', ...
'--dataset_path=', datasetPath, ...
' --vio_params_path=', filenameVioParams, ...
' --tracker_params_path=',  filenameTrackerParams, ...,
' --backend_type=', backend_type
' --log_output=true', ...
' --visualize=false', ...
' --viz_type=5', ' > out-', num2str(i),'.txt');

myCommand = sprintf(myCommand, sudoName, appName);

disp(myCommand)
system(myCommand)

%% Unfortunately, to run this in Matlab (only under ubuntu, we need to sudo)

%% Only for debugging
% system(horzcat('../build/stereoVIOExample ',datasetPath,' > out-',num2str(i),'.txt'));
% system(horzcat('sudo ../build/tests/testFrame'));

fprintf('Completed VIO run nr %d \n',i);