% -------------------------------------------------------------------------------------------------
function bboxes = tracker(varargin)
%TRACKER
%   is the main function that performs the tracking loop
%   Default parameters are overwritten by VARARGIN
%
%   Luca Bertinetto, Jack Valmadre, Joao F. Henriques, 2016
% -------------------------------------------------------------------------------------------------
    % These are the default hyper-params for SiamFC-3S
    % The ones for SiamFC (5 scales) are in params-5s.txt
    
    ENABLE_KALMAN = 1;
    ENABLE_TEMPLATE_UPDATE = 1;
    
    p.numScale = 3;
    p.scaleStep = 1.0375;
    p.scalePenalty = 0.9745;
    p.scaleLR = 0.59; % damping factor for scale update
    p.responseUp = 16; % upsampling the small 17x17 response helps with the accuracy
    p.windowing = 'cosine'; % to penalize large displacements
    p.wInfluence = 0.176; % windowing influence (in convex sum)
    p.net = '2016-08-17.net.mat';
    %% execution, visualization, benchmark
    p.video = 'vot15_bag';
    p.visualization = false;
    p.gpus = 0;
    p.bbox_output = false;
    p.fout = -1;
    %% Params from the network architecture, have to be consistent with the training
    p.exemplarSize = 127;  % input z size
    p.instanceSize = 255;  % input x size (search region)
    p.scoreSize = 17;
    p.totalStride = 8;
    p.contextAmount = 0.5; % context amount for the exemplar
    p.subMean = false;
    %% SiamFC prefix and ids
    p.prefix_z = 'a_'; % used to identify the layers of the exemplar
    p.prefix_x = 'b_'; % used to identify the layers of the instance
    p.prefix_join = 'xcorr';
    p.prefix_adj = 'adjust';
    p.id_feat_z = 'a_feat';
    p.id_score = 'score';
    % Overwrite default parameters with varargin
    p = vl_argparse(p, varargin);
% -------------------------------------------------------------------------------------------------
    % Get environment-specific default paths.
    p = env_paths_tracking(p);
    % Load ImageNet Video statistics
    if exist(p.stats_path,'file')
        stats = load(p.stats_path);
    else
        warning('No stats found at %s', p.stats_path);
        stats = [];
    end
    % Load two copies of the pre-trained network
    net_z = load_pretrained([p.net_base_path p.net], []);
    net_x = load_pretrained([p.net_base_path p.net], []);
    [imgFiles, targetPosition, targetSize] = load_video_info(p.seq_base_path, p.video);
    nImgs = numel(imgFiles);
    startFrame = 1;
    % Divide the net in 2
    % exemplar branch (used only once per video) computes features for the target
    remove_layers_from_prefix(net_z, p.prefix_x);
    remove_layers_from_prefix(net_z, p.prefix_join);
    remove_layers_from_prefix(net_z, p.prefix_adj);
    % instance branch computes features for search region x and cross-correlates with z features
    remove_layers_from_prefix(net_x, p.prefix_z);
    zFeatId = net_z.getVarIndex(p.id_feat_z);
    scoreId = net_x.getVarIndex(p.id_score);
    % get the first frame of the video
    im = imgFiles{startFrame}; %gpuArray(single(imgFiles{startFrame}));
    % if grayscale repeat one channel to match filters size
	if(size(im, 3)==1)
        im = repmat(im, [1 1 3]);
    end
    % Init visualization
    videoPlayer = [];
    if p.visualization && isToolboxAvailable('Computer Vision System Toolbox')
        videoPlayer = vision.VideoPlayer('Position', [100 100 [size(im,2), size(im,1)]+30]);
    end
    % get avg for padding
    avgChans = gather([mean(mean(im(:,:,1))) mean(mean(im(:,:,2))) mean(mean(im(:,:,3)))]);
    
    wc_z = targetSize(2) + p.contextAmount*sum(targetSize);
    hc_z = targetSize(1) + p.contextAmount*sum(targetSize);
    s_z = sqrt(wc_z*hc_z);
    s_z_original = s_z;
    scale_z = p.exemplarSize / s_z;
    % initialize the exemplar
    [z_crop, ~] = get_subwindow_tracking(im, targetPosition, [p.exemplarSize p.exemplarSize], [round(s_z) round(s_z)], avgChans);
    if p.subMean
        z_crop = bsxfun(@minus, z_crop, reshape(stats.z.rgbMean, [1 1 3]));
    end
    z_crop_original=z_crop;
    d_search = (p.instanceSize - p.exemplarSize)/2;
    pad = d_search/scale_z;
    s_x = s_z + 2*pad;
    s_x_original = s_x;
    % arbitrary scale saturation
    min_s_x = 0.2*s_x;
    min_s_x_original = min_s_x;
    min_s_x_original_old = min_s_x;
    max_s_x = 5*s_x;
    max_s_x_original = max_s_x;
    max_s_x_original_old = max_s_x;
    if ENABLE_KALMAN
        [A, B, u, H, P_k, R, Q, x] = kalmanInit([targetPosition, targetSize]);
    end
    switch p.windowing
        case 'cosine'
            window = single(hann(p.scoreSize*p.responseUp) * hann(p.scoreSize*p.responseUp)');
        case 'uniform'
            window = single(ones(p.scoreSize*p.responseUp, p.scoreSize*p.responseUp));
    end
    % make the window sum 1
    window = window / sum(window(:));
    scales = (p.scaleStep .^ ((ceil(p.numScale/2)-p.numScale) : floor(p.numScale/2)));
    % evaluate the offline-trained network for exemplar z features
    net_z.eval({'exemplar', z_crop});
    z_features = net_z.vars(zFeatId).value;
    z_features = repmat(z_features, [1 1 1 p.numScale]);
    
    z_features_orig = z_features;
    z_features_orig_old = z_features;
    bboxes = zeros(nImgs, 4);
    % start tracking
    tic;
    for i = startFrame:nImgs
        if i>startFrame
            % load new frame on GPU
            im = imgFiles{i}; %gpuArray(single(imgFiles{i}));
   			% if grayscale repeat one channel to match filters size
    		if(size(im, 3)==1)
        		im = repmat(im, [1 1 3]);
            end
            
            if ENABLE_TEMPLATE_UPDATE && i>startFrame+0 %(mod(i, 1) == 0)%startFrame+10
                wc_z = targetSize(2) + p.contextAmount*sum(targetSize);
                hc_z = targetSize(1) + p.contextAmount*sum(targetSize);
                s_z = sqrt(wc_z*hc_z);

                
                % initialize the exemplar
                [z_crop, ~] = get_subwindow_tracking(im, targetPosition, [p.exemplarSize p.exemplarSize], [round(s_z) round(s_z)], avgChans);
                if p.subMean
                    z_crop = bsxfun(@minus, z_crop, reshape(stats.z.rgbMean, [1 1 3]));
                end
        
                net_z.eval({'exemplar', z_crop});
                z_features = net_z.vars(zFeatId).value;
                z_features = repmat(z_features, [1 1 1 p.numScale]);
                
                correlation_coeff = corr2(double(z_features(:)), double(z_features_orig(:)))
%                 correlation_coeff = 1;
                if correlation_coeff < 0.8 && correlation_coeff > 0.7
                    z_features = z_features_orig;
%                     s_x = s_x_original;
                    min_s_x = min_s_x_original;
                    max_s_x = max_s_x_original;
                    
                elseif correlation_coeff <=0.7
%                     s_x = s_x_original;
                    z_features = z_features_orig_old;
                    min_s_x = min_s_x_original_old;
                    max_s_x = max_s_x_original_old;
                        
                else
                    
                    scale_z = p.exemplarSize / s_z;
                    d_search = (p.instanceSize - p.exemplarSize)/2;
                    pad = d_search/scale_z;
                    s_x = s_z + 2*pad;
                    % arbitrary scale saturation
                    min_s_x = 0.2*s_x;
                    max_s_x = 5*s_x;
                    %% TO CHANGE/UPDATE ORIGINAL FEATURES TO THE NEWEST ONE?
                    z_features_orig = z_features;
                    min_s_x_original = min_s_x;
                    max_s_x_original = max_s_x;
                end
                
            end
            scaledInstance = s_x .* scales;
            scaledTarget = [targetSize(1) .* scales; targetSize(2) .* scales];
            % extract scaled crops for search region x at previous target position
            x_crops = make_scale_pyramid(im, targetPosition, scaledInstance, p.instanceSize, avgChans, stats, p);
            % evaluate the offline-trained network for exemplar x features
            [newTargetPosition, newScale] = tracker_eval(net_x, round(s_x), scoreId, z_features, x_crops, targetPosition, window, p);
            targetPosition = gather(newTargetPosition);
            % scale damping and saturation
            s_x = max(min_s_x, min(max_s_x, (1-p.scaleLR)*s_x + p.scaleLR*scaledInstance(newScale)));
            targetSize = (1-p.scaleLR)*targetSize + p.scaleLR*[scaledTarget(1,newScale) scaledTarget(2,newScale)];
        else
            % at the first frame output position and size passed as input (ground truth)
        end

        rectPosition = [targetPosition([2,1]) - targetSize([2,1])/2, targetSize([2,1])];
        % output bbox in the original frame coordinates
        oTargetPosition = targetPosition; % .* frameSize ./ newFrameSize;
        oTargetSize = targetSize; % .* frameSize ./ newFrameSize;
        bboxes(i, :) = [oTargetPosition([2,1]) - oTargetSize([2,1])/2, oTargetSize([2,1])];

        
        %*********************************************
        if ENABLE_KALMAN
            kalmanInput          = targetPosition';
            [kalmanOutput, P_k, x] = kalmanFilter(A, B, u, H, P_k, R, Q, x, kalmanInput); 
            correctedROI = [kalmanOutput', targetSize];
            targetPosition = correctedROI(1:2);
            targetSize = correctedROI(3:4);
            correctedRectPos = [correctedROI(2)-correctedROI(4)/2, correctedROI(1)-correctedROI(3)/2, correctedROI(4), correctedROI(3)];
        end
        %*********************************************
        
        if p.visualization
            if isempty(videoPlayer)
                figure(1), imshow(im/255);
                figure(1), rectangle('Position', rectPosition, 'LineWidth', 4, 'EdgeColor', 'y');
                figure(2), imshow(uint8(z_crop), []);
                text(rectPosition(1), rectPosition(2)+rectPosition(4)-40, ...
                    'SiamFC', 'Color','yellow', 'FontSize', 14, 'FontWeight', 'bold');
                
                if ENABLE_KALMAN
                    figure(1), rectangle('Position', correctedRectPos, 'LineWidth', 4, 'EdgeColor', 'g');
                    text(correctedRectPos(1), correctedRectPos(2)-30, ...
                    'Kalman', 'Color','green', 'FontSize', 14, 'FontWeight', 'bold');
                end
                drawnow
%                 fprintf('Frame %d\n', startFrame+i);
            else
                im = gather(im)/255;
                im = insertShape(im, 'Rectangle', rectPosition, 'LineWidth', 4, 'Color', 'yellow');
                % Display the annotated video frame using the video player object.
                step(videoPlayer, im);
            end
        end

        if p.bbox_output
            fprintf(p.fout,'%.2f,%.2f,%.2f,%.2f\n', bboxes(i, :));
        end
        
%         F1(i+1) = getframe(gcf);
    end

    bboxes = bboxes(startFrame : i, :);
%     F1 = F1(:, 2:end);
%     writeVideo(F1, 'siamFC_Kalman');
end
