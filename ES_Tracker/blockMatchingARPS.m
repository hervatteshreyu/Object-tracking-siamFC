%--------------------------------------------------------------------------
%% Block Matching : Adaptive Rood Pattern Search 
%--------------------------------------------------------------------------
%  
% This function implements the ARPS block matching and returns the
% displacemnt vector.
%
% [in] : imgP (current image)
% [in] : imgI (reference image)
% [in] : ROI (referene ROI to be matched)
% [in] : vectors (displacement vectors)
%
% [out] : motionVect (updated displacement vector)
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author : Skanda Bharadwaj 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%
function [motionVect, computations] = blockMatchingARPS(imgP, imgI, ROI, ...
                                                        vectors)

    SUBPIXEL_ENABLED = 1;
    
    % Initialize macroblock size
    mbSize = [ROI(3), ROI(4)];
    
    % Initialize search region
    p = 20; %round(max(mbSize)/2);

    [row, col] = size(imgI);

    % Initialize costs at search patterns of the diamond
    costs = ones(1, 6) * 1e-13;

    % Initialize small diamond search kernel
    SDSP(1,:) = [ 0 -1];
    SDSP(2,:) = [-1  0];
    SDSP(3,:) = [ 0  0];
    SDSP(4,:) = [ 1  0];
    SDSP(5,:) = [ 0  1];

    % Initialize check matrix (to track where search is already carried out)
    checkMatrix = zeros(2*p+1,2*p+1);

    % Stores original coordinates 
    i = ROI(2);
    j = ROI(1);

    % Stores the upadted value of the coordinates
    x = j;
    y = i;

    % Create the subimage of macroblock size for the search
    subImgI = imgI(i:i+mbSize(2)-1, j:j+mbSize(1)-1);
    subImgP = imgP(i:i+mbSize(2)-1, j:j+mbSize(1)-1);
    
    % Update the cost(3) center of the diamond
    costs(3) = costFuncNCC(subImgP, subImgI);
    
    % Update the check matrix
    checkMatrix(p+1, p+1) = 1;
    
    % Keeps track of number of computations
    computations = 0;

    %% Perfrom LDSP
    
    % At the boundaries
    if (j-1 < 1)
        stepSize = 2;
        maxIndex = 5;
    else
        stepSize = max(abs(vectors(1, 1)), abs(vectors(2, 1)));

        if ( (abs(vectors(1, 1)) == stepSize && vectors(2, 1) == 0) ...
                || (abs(vectors(2, 1)) == stepSize && vectors(1, 1) == 0)) ...

            maxIndex = 5; % we just have to check at the rood pattern 5 points

        else
            maxIndex = 6; % we have to check 6 points
            LDSP(6,:) = [vectors(2, 1)  vectors(1, 1)];
        end
    end

    % Initialize LDSP kernel
    LDSP(1,:) = [        0  -stepSize];
    LDSP(2,:) = [-stepSize          0];
    LDSP(3,:) = [        0          0];
    LDSP(4,:) = [ stepSize          0];
    LDSP(5,:) = [        0   stepSize];


    % Large diamond search
    for k = 1:maxIndex
        refBlkVer = y + LDSP(k,2);   % row/Vert co-ordinate for ref block
        refBlkHor = x + LDSP(k,1);   % col/Horizontal co-ordinate
        if ( refBlkVer < 1 || refBlkVer+mbSize(2)-1 > row ...
                || refBlkHor < 1 || refBlkHor+mbSize(1)-1 > col)

            continue; % outside image boundary
        end

        if (k == 3 || stepSize == 0)
            continue; % center point already calculated
        end
        
        costs(k) = costFuncNCC(imgP(i:i+mbSize(2)-1,j:j+mbSize(1)-1), ...
                imgI(refBlkVer:refBlkVer+mbSize(2)-1, refBlkHor:refBlkHor+mbSize(1)-1));
        
        checkMatrix(round([LDSP(k,2) + p+1, LDSP(k,1) + p+1])) = 1;
        computations = computations + 1;

    end

    % Find minimum cost and index
    [cost, point] = max(costs);

    % Update the coordinated of the matched block
    x = x + LDSP(point, 1);
    y = y + LDSP(point, 2);
    
    % Reinitialize costs for SDSP
    costs = ones(1,5) * 1e-13;
    
    % Bring the small diamond to the center
    costs(3) = cost;

    % perform SDSP
    doneFlag = 0;
    while (doneFlag == 0)
        for k = 1:5
            refBlkVer = y + SDSP(k,2);   % row/Vert co-ordinate for ref block
            refBlkHor = x + SDSP(k,1);   % col/Horizontal co-ordinate
            if ( refBlkVer < 1 || refBlkVer+mbSize(2)-1 > row ...
                    || refBlkHor < 1 || refBlkHor+mbSize(1)-1 > col)
                continue;
            end

            if (k == 3)
                continue
            elseif (refBlkHor < j-p || refBlkHor > j+p || refBlkVer < i-p  ...
                    || refBlkVer > i+p)
                continue;
            elseif (checkMatrix(round([y-i+SDSP(k,2)+p+1, x-j+SDSP(k,1)+p+1])) == 1)
                continue
            end
            
            costs(k) = costFuncNCC(imgP(i:i+mbSize(2)-1,j:j+mbSize(1)-1), ...
                imgI(refBlkVer:refBlkVer+mbSize(2)-1, refBlkHor:refBlkHor+mbSize(1)-1));
                                    
            checkMatrix(round([y-i+SDSP(k,2)+p+1, x-j+SDSP(k,1)+p+1])) = 1;
            computations = computations + 1;
        end

        % Find minimum cost and index
        [cost, point] = max(costs);

        % Check if the best match is at the center of the diamond
        if (point == 3)
            doneFlag = 1;
            
            refBlkVer = y + SDSP(point, 2);
            refBlkHor = x + SDSP(point, 1);
            if SUBPIXEL_ENABLED
                interpolationCoorinates = getSubpixelMatch(refBlkVer, refBlkHor, ...
                                               imgI, imgP, mbSize, i, j, x, y);  
            else
                interpolationCoorinates = [y, x];
            end
                                    
        % if not, update best match and continue SDSP
        else
            x = x + SDSP(point, 1);
            y = y + SDSP(point, 2);
            costs = ones(1,5) * 1e-13;
            costs(3) = cost;
        end

    end
    warning('off','all');
    % Fectch the displacement of the best match
    vectors(1, 1) = interpolationCoorinates(1)-i;    
    vectors(2, 1) = interpolationCoorinates(2)-j;   

    motionVect = vectors;
    
end

%--------------------------------------------------------------------------
%% END