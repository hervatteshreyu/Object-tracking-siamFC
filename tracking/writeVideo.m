%--------------------------------------------------------------------------
%% Write Video
%--------------------------------------------------------------------------
%  
% This function creates a video given a set of frames.
%
% [in] : frames (sequence of frames)
% [in] : videoName (string input) 
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author : Skanda Bharadwaj 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%
function writeVideo(frames, videoName)
    
    writerObj = VideoWriter(videoName);
    writerObj.FrameRate = 5;

    % set the seconds per image
    % open the video writer

    open(writerObj);
    
    % write the frames to the video
    for i=1:length(frames)
        
        % convert the image to a frame
        frame = frames(i) ;    
        writeVideo(writerObj, frame);
    end
    
    % close the writer object
    close(writerObj);    
end

%--------------------------------------------------------------------------
%% END