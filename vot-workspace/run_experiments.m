% This script can be used to execute the experiments for a single tracker
% You can copy and modify it to create another experiment launcher

addpath('/Users/skandabharadwaj/Documents/MATLAB/vot-toolkit-master'); toolkit_path; % Make sure that VOT toolkit is in the path

[sequences, experiments] = workspace_load();

tracker = tracker_load('siamKalman');

workspace_evaluate(tracker, sequences, experiments);

