function [vectors, computations] = blockMatchingExhaustiveSearch(imgP, imgI,...
                                                              ROI, vectors)
    
    SUBPIXEL_ENABLED = 0;
    
    % Initialize macroblock size
    mbSize = [ROI(3), ROI(4)];
    
    % Initialize search region
    p = 20; %round(max(mbSize)/2);

    [row, col] = size(imgI);

    % Initialize Cost matrix
    costs = ones(2*p + 1, 2*p +1) * 1e-13;
    
    % Keeps track of number of computations
    computations = 0;

    i = ROI(2);
    j = ROI(1);
    
    % Perfrom Exhaustive search and update the cost matrix 
    for m = -p : p        
        for n = -p : p
            refBlkVer = i + m;   
            refBlkHor = j + n;   
            if ( refBlkVer < 1 || refBlkVer+mbSize(2)-1 > row ...
                    || refBlkHor < 1 || refBlkHor+mbSize(1)-1 > col)
                continue;
            end
            subImgP = imgP(i:i+mbSize(2)-1,j:j+mbSize(1)-1);
            subImgI = imgI(refBlkVer:refBlkVer+mbSize(2)-1, ...
                           refBlkHor:refBlkHor+mbSize(1)-1);
                       
            costs(m+p+1, n+p+1) = costFuncNCC(subImgP, subImgI);
            computations = computations + 1;
        end
    end
   
    % Calculate the max cost
    maxCost  = max(max(abs(costs)));
    [maxAxial, maxLateral] = find(abs(costs) == maxCost);
    
    dy = maxAxial-(p+1);
    dx = maxLateral-(p+1);
    
    axial_frac   = 0;
    lateral_frac = 0;
    
    if SUBPIXEL_ENABLED
        if (maxAxial == 1)     ||  ...
           (maxAxial == 2*p+1) ||  ...
           (maxLateral ==1)    ||  ...
           (maxLateral == 2*p+1)
            dx = 0;
            dz = 0;
        return;
        end
        cc_mag = abs(costs(maxAxial-1:maxAxial+1, maxLateral-1:maxLateral+1));    
        [xmesh, ymesh] = meshgrid(-1:1);
        interp_lateral = 100;
        interp_axial = 10;

        [xmesh_interp, ymesh_interp] = meshgrid(-1:1/interp_lateral:1, ...
                                                -1:1/interp_axial:1);

        cc_mag_interp = interp2(xmesh, ymesh, cc_mag, xmesh_interp, ...
                                ymesh_interp, 'cubic');    

        cc_mag_interp_max = max(max(cc_mag_interp));
        [InterpMaxIndex_axial, InterpMaxIndex_lateral] = ...
            find(cc_mag_interp == cc_mag_interp_max);

        axial_frac   = (InterpMaxIndex_axial-1)*1/interp_axial-1;
        lateral_frac = (InterpMaxIndex_lateral-1)*1/interp_lateral-1;
    end
    
    % Find the motion vectors
    vectors(1, 1) = dy + axial_frac;    
    vectors(2, 1) = dx + lateral_frac;    
end
                    