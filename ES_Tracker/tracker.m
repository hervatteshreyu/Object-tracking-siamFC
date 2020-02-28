%--------------------------------------------------------------------------
%% Motion Estimation : ARPS Block Matching with Kalman Filter
%--------------------------------------------------------------------------
%  Implementation of the motion estimation of the wall-lumen in the common
%  carotid artery using block matching with kalman filtering. Adaptive Rood
%  Patter Search (ARPS) is the block matching algorithm used in this
%  implementaiton. 
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author : Skanda Bharadwaj 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% ========================================================================

clear; close all; clc;

%% ========================================================================

% Load data
imageName = 'caltrain';

%% ========================================================================

ENABLE_KALMAN            = 1;
ENBLE_ADAPTIVE_SEARCH    = 0;
ENABLE_EXHAUSTIVE_SEARCH = 1;

if ENABLE_KALMAN
    kalman = 'Kalman';
    fprintf('Kalman Enabled\n');
else
    kalman = '';
    fprintf('Kalman Disabled\n');
end

if ENBLE_ADAPTIVE_SEARCH
    SearchAlgo = 'ARPS';
    fprintf('Search Algorithm : Adaptive Rood Pattern Search (ARPS)\n\n');
end

if ENABLE_EXHAUSTIVE_SEARCH
    SearchAlgo = 'ES';
    fprintf('Search Algorithm : Exhaustive Search (ES)\n\n');
end

if ~(ENABLE_EXHAUSTIVE_SEARCH || ENBLE_ADAPTIVE_SEARCH)
    SearchAlgo = '';
    fprintf('Search Algorithm : Disabled\n\n');
end

videoPath = '../demo-sequences/truckSign/imgs/';
imgDir = dir([videoPath, '*.png']);
nFrames = size(imgDir, 1);
firstFrame = rgb2gray(imread([videoPath, imgDir(1).name]));

% Get the user input in the first frame
[userInputROI, imgHeight, imgWidth] = getUserROI(firstFrame);

% Uncomment the below line for default ROI.
% userInputROI = [288.94,249.48,79.04,83.97];
firstFrameROI = userInputROI;
fprintf('User input fetched successfully.\n\n');


% Initialize Kalman Filter
if ENABLE_KALMAN
    [A, B, u, H, P, R, Q, x] = kalmanInit(userInputROI);
end

%% Block Matching and Kalman Filter
motionVect   = zeros(2, 1);
trackedROI   = zeros(nFrames-1, 4);
correctedROI = zeros(nFrames-1, 4);
displacementTrajectory = zeros(nFrames, 2);

frameCnt = 0; 
timePerFrame = zeros(nFrames-1, 1);
computationsPerFrame = zeros(nFrames-1, 1);

for i = 1:nFrames-2
    frameCnt = frameCnt + 1;
    fprintf('Processing frame : %d\n', i+1);
    
    imgINumber = i;
    imgPNumber = i+1;
    
    imgR = double(rgb2gray(imread([videoPath, imgDir(i).name])));
    imgC = double(rgb2gray(imread([videoPath, imgDir(i+1).name])));
    
%     imgR = double(imread(imgIFile));
%     imgC = double(imread(imgPFile));
  
    % ARPS function call
    if ENBLE_ADAPTIVE_SEARCH
        t1 = tic;
        [motionVect, ARPSComputations] = ...
                   blockMatchingARPS(imgC, imgR, userInputROI, motionVect);
        timePerFrame(i, 1) = toc(t1);
        computationsPerFrame(i, 1) = ARPSComputations;
    end
    
    % Exhaustive Search function call
    if ENABLE_EXHAUSTIVE_SEARCH
        t1 = tic;
        [motionVect, ESComputations] = ...
                blockMatchingExhaustiveSearch(imgC, imgR, userInputROI, ...
                                              motionVect);
         timePerFrame(i, 1) = toc(t1);
         computationsPerFrame(i, 1) = ESComputations;
    end
    
    displacementTrajectory(i, :) = motionVect';
    
    % Update the matched ROI
    trackedROI(frameCnt, :) = [userInputROI(1)-motionVect(2, 1), ...
                               userInputROI(2)-motionVect(1, 1), ...
                               userInputROI(3:4)];
    
    correctedROI(frameCnt, :) = trackedROI(frameCnt, :);
    
    % Call Kalman Filter
    if ENABLE_KALMAN
        kalmanInput          = trackedROI(frameCnt, 1:2)';
        [kalmanOutput, p, x] = kalmanFilter(A, B, u, H, P, R, Q, x, ...
                                            kalmanInput); 
    
        % Update the tracked ROI with corrected ROI
        correctedROI(frameCnt, :) = [kalmanOutput', trackedROI(frameCnt, 3:4)];
        motionVect(2, 1)   =  userInputROI(1)-correctedROI(frameCnt, 1);
        motionVect(1, 1)   =  userInputROI(2)-correctedROI(frameCnt, 2);
        displacementTrajectory(frameCnt, :) = motionVect';
    end
    
    % Visulalization
    figure(1); 
    imshow(imgC, []);  hold on;
    
    if ENBLE_ADAPTIVE_SEARCH
        rectangle('Position', trackedROI(frameCnt, :), 'Edgecolor', 'r', 'Linewidth', 3);
        text(trackedROI(frameCnt, 1), trackedROI(frameCnt, 2)+trackedROI(frameCnt, 4)+10, ...
             'ARPS', 'Color','red', 'FontSize', 12);
    end
    
    if ENABLE_EXHAUSTIVE_SEARCH
        rectangle('Position', trackedROI(frameCnt, :), 'Edgecolor', 'r', 'Linewidth', 3);
        text(trackedROI(frameCnt, 1), trackedROI(frameCnt, 2)+trackedROI(frameCnt, 4)+10, ...
             'ES', 'Color','red', 'FontSize', 12);
    end
    
    if ENABLE_KALMAN
        rectangle('Position', correctedROI(frameCnt, :), 'Edgecolor', 'g', 'Linewidth', 3);
        text(correctedROI(frameCnt, 1), correctedROI(frameCnt, 2)-10, ...
             'Kalman', 'Color','green', 'FontSize', 12);
    end
    
    text1 = sprintf('%s %s Tracking', SearchAlgo, kalman);
    title(text1, 'FontSize', 14', 'Fontweight', 'bold');
    hold off;
    
    userInputROI = correctedROI(frameCnt, :);
    warning('off','all');
end

averageTimePerFrame = sum(timePerFrame)/frameCnt;
fprintf('Average computational-Time/frame for %s is %0.02fs\n\n', ...
         SearchAlgo, averageTimePerFrame);
     
averageComputationsPerFrame = sum(computationsPerFrame)/frameCnt;
fprintf('Average computations/frame for %s is %0.02f\n\n', ...
         SearchAlgo, averageComputationsPerFrame);

centroids = getStatics(firstFrameROI, trackedROI, correctedROI);

%% Figures

figure()
bar(timePerFrame);
xlabel('nFrames', 'FontSize', 14', 'Fontweight', 'bold'); 
ylabel('Time/frame', 'FontSize', 14', 'Fontweight', 'bold');
text = sprintf('Framewise computational time for %s', SearchAlgo);
title(text, 'Fontweight', 'Bold', 'Fontsize', 16);

figure()
bar(computationsPerFrame);
xlabel('nFrames', 'FontSize', 14', 'Fontweight', 'bold'); 
ylabel('Computatiosn/frame', 'FontSize', 14', 'Fontweight', 'bold');
text = sprintf('Framewise computations for %s', SearchAlgo);
title(text, 'Fontweight', 'Bold', 'Fontsize', 16);

figure()
subplot(211); 
plot(displacementTrajectory(:, 1));
xlabel('nFrames', 'FontSize', 14', 'Fontweight', 'bold'); 
ylabel('Axial displacement', 'FontSize', 14', 'Fontweight', 'bold');
legend('Predicted', 'Groundtruth');
grid on; hold off;

subplot(212); 
plot(displacementTrajectory(:, 2));
xlabel('nFrames', 'FontSize', 14', 'Fontweight', 'bold'); 
ylabel('Lateral displacement', 'FontSize', 14', 'Fontweight', 'bold');
legend('Predicted', 'Groundtruth');
grid on; hold off;

suptitle('Displacement trajectory of the ROI');


%--------------------------------------------------------------------------
%% END