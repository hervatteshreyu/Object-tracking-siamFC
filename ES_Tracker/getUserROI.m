%--------------------------------------------------------------------------
%% Get User ROI
%--------------------------------------------------------------------------
%  
% This function allows user to select a ROI that is to be tracked.
%
% [in] : Frame (the first frame of the video sequence
%
% [out] : rect(bounding box of the ROI (x, y, w, h))
% [out] : M, N (size of the image)
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author : Skanda Bharadwaj 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%
function [rect, M, N] = getUserROI(frame)
    
    % Read image
    [M, N] = size(frame);
    
    % Display image
    imshow(frame, []);
    title('Select object to be tracked', 'FontSize', 14', 'Fontweight', 'bold');
    
    % Fetch the ROI
    rect = getrect();
end

%--------------------------------------------------------------------------
%% END