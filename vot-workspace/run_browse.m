% This script can be used to interactively inspect the results

addpath('/Users/skandabharadwaj/Documents/MATLAB/vot-toolkit-master'); toolkit_path; % Make sure that VOT toolkit is in the path

[sequences, experiments] = workspace_load();

trackers = tracker_load('siamKalman');

workspace_browse(trackers, sequences, experiments);

