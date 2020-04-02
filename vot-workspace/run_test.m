% This script can be used to test the integration of a tracker to the
% framework.

addpath('/Users/skandabharadwaj/Documents/MATLAB/vot-toolkit-master'); toolkit_path; % Make sure that VOT toolkit is in the path

[sequences, experiments] = workspace_load();

tracker = tracker_load('siamKalman');

workspace_test(tracker, sequences);

