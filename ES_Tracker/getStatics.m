%--------------------------------------------------------------------------
%% Block Matching : Adaptive Rood Pattern Search 
%--------------------------------------------------------------------------
%  
% This function implements calculation of statics on the groundtruth,
% tracked and corrected RoI regions. It calculates and plots centroid
% errors, RMSE the bounding box overlap over all frames.
% 
% [in] : firstFrameROI (The RoI selected by the user)
% [in] : groundTruthROI
% [in] : trackedROI (RoI calculated by the search algorithm alone)
% [in] : correcteddROI (RoI corrected by the by the kalman filter)
%
% [out] : centroidError (Centroid error for each frame in both lateral and 
%                        axial directions and RMSE)
% [out] : bboxOverlap (bounding box overlap for each frame (GT and tracked)
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author : Skanda Bharadwaj 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%
function [centroids] = getStatics(firstFrameROI, trackedROI, correctedROI)
    
    
    TRoI = [firstFrameROI; trackedROI];
    CRoI = [firstFrameROI; correctedROI];
    
    TRoI_Centroids = [TRoI(:, 1)+(TRoI(:, 3)/2), TRoI(:, 2)+(TRoI(:, 4)/2)]; 
    CRoI_Centroids = [CRoI(:, 1)+(CRoI(:, 3)/2), CRoI(:, 2)+(CRoI(:, 4)/2)];
    
    centroids = [TRoI_Centroids, CRoI_Centroids];
    
    % Plot centroids
    figure(); 
    scatter(TRoI_Centroids(1:end-1, 1), TRoI_Centroids(1:end-1, 2), 'r', 'Linewidth', 2); hold on;
    scatter(CRoI_Centroids(1:end-1, 1), CRoI_Centroids(1:end-1, 2), 'b', 'Linewidth', 2);
    legend('Block Matching', 'Kalman Correction');
    xlabel('Lateral', 'Fontweight', 'Bold', 'Fontsize', 12); 
    ylabel('Axial', 'Fontweight', 'Bold', 'Fontsize', 12);
    title('Centroids of RoI', 'Fontweight', 'Bold', 'Fontsize', 14);
    grid on; hold off;
end