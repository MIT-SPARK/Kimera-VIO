function isEqual = resultsEqual(results1, results2,relTol)

zeroTol = 1e-5;
isEqual = true;

%% check mono ransac errors
if ~isequalWithTol(results1.maxRelRotErrors_mono,results2.maxRelRotErrors_mono,relTol,zeroTol)
    warning('maxRelRotErrors_mono mismatch');
    isEqual = false;
end
if ~isequalWithTol(results1.meanRelRotErrors_mono,results2.meanRelRotErrors_mono,relTol,zeroTol)
    warning('meanRelRotErrors_mono mismatch');
    isEqual = false;
end
if ~isequalWithTol(results1.maxRelTranErrors_mono,results2.maxRelTranErrors_mono,relTol,zeroTol)
    warning('maxRelTranErrors_mono mismatch');
    isEqual = false;
end
if ~isequalWithTol(results1.meanRelTranErrors_mono,results2.meanRelTranErrors_mono,relTol,zeroTol)
    warning('meanRelTranErrors_mono mismatch');
    isEqual = false;
end

%% check stereo ransac errors
if ~isequalWithTol(results1.maxRelRotErrors_stereo,results2.maxRelRotErrors_stereo,relTol,zeroTol)
    warning('maxRelRotErrors_stereo mismatch');
    isEqual = false;
end
if ~isequalWithTol(results1.meanRelRotErrors_stereo,results2.meanRelRotErrors_stereo,relTol,zeroTol)
    warning('meanRelRotErrors_stereo mismatch');
    isEqual = false;
end
if ~isequalWithTol(results1.maxRelTranErrors_stereo,results2.maxRelTranErrors_stereo,relTol,zeroTol)
    warning('maxRelTranErrors_stereo mismatch');
    isEqual = false;
end
if ~isequalWithTol(results1.meanRelTranErrors_stereo,results2.meanRelTranErrors_stereo,relTol,zeroTol)
    warning('meanRelTranErrors_stereo mismatch');
    isEqual = false;
end

%% check vio errors
if ~isequalWithTol(results1.maxRotErrors_vio,results2.maxRotErrors_vio,relTol,zeroTol)
    warning('maxRotErrors_vio mismatch');
    isEqual = false;
end
if ~isequalWithTol(results1.meanRotErrors_vio,results2.meanRotErrors_vio,relTol,zeroTol)
    warning('meanRotErrors_vio mismatch');
    isEqual = false;
end
if ~isequalWithTol(results1.maxTranErrors_vio,results2.maxTranErrors_vio,relTol,zeroTol)
    warning('maxTranErrors_vio mismatch');
    isEqual = false;
end
if ~isequalWithTol(results1.meanTranErrors_vio,results2.meanTranErrors_vio,relTol,zeroTol)
    warning('meanTranErrors_vio mismatch');
    isEqual = false;
end


