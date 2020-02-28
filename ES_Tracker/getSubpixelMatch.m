%--------------------------------------------------------------------------
%% Block Matching : Adaptive Rood Pattern Search 
%--------------------------------------------------------------------------
%  
% This function implements interpolation 
% 
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author : Skanda Bharadwaj 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%
function [interpolationCoorinates]  = getSubpixelMatch(refBlkVer, refBlkHor, ...
                                        imgI, imgP, mbSize, ii, jj, x, y)
    
    
    % Move the reference frame around the current frame 
    axial   = -1:1;
    lateral = -1:1;
    
    cc = zeros(length(axial));
    for i = 1:length(axial)
        for j = 1:length(lateral)
            refVer = refBlkVer + axial(i);
            refHor = refBlkHor + lateral(j);
          
            cc(i, j) = costFuncNCC(imgP(ii:ii+mbSize(2)-1, jj:jj+mbSize(1)-1),   ...
                                   imgI(refVer:refVer+mbSize(2)-1,      ...
                                        refHor:refHor+mbSize(1)-1));
            warning('off','all');
        end
    end
    
    cc_mag = abs(cc);
    
    
    interp_axial = 1;
    interp_lateral = 10;
    
    [xmesh, ymesh] = meshgrid(-1:1);
    [xmesh_interp, ymesh_interp] = meshgrid(-1:1/interp_lateral:1, ...
                                            -1:1/interp_axial:1);
    
    cc_mag_interp = interp2(xmesh, ymesh, cc_mag, xmesh_interp, ...
                            ymesh_interp, 'cubic');    
    
    cc_mag_interp_max = max(max(cc_mag_interp));
    [InterpMaxIndex_axial, InterpMaxIndex_lateral] = ...
                                  find(cc_mag_interp == cc_mag_interp_max);
    
                             
    axial_frac   = ((InterpMaxIndex_axial-1)/interp_axial)-1;
    lateral_frac = ((InterpMaxIndex_lateral-1)/interp_lateral)-1;
    
    interp_axial   = y + axial_frac;
    interp_lateral = x + lateral_frac;
    
    interpolationCoorinates = [interp_axial, interp_lateral];
    
%     A = [1  1  1  -1  -1  1;
%          1  0  0  -1   0  1;
%          1  1 -1  -1   1  1;
%          0  1  0   0  -1  1;
%          0  0  0   0   0  1;
%          0  1  0   0   1  1;
%          1  1 -1   1  -1  1;
%          1  0  0   1   0  1;
%          1  1  1   1   1  1;
%          ];
%     
%     P_A = pinv(A'*A)*A';
%     f = reshape(cc_mag, 9, 1);
%     b = P_A*f;
%     
%     dx_f = (-2*b(2)*b(4)+b(3)*b(5))/(4*b(1)*b(2)-b(3)^2);
%     dy_f = (-2*b(1)*b(5)+b(3)*b(4))/(4*b(1)*b(2)-b(3)^2);
%     
%     dx = x + dx_f;
%     dy = y + dy_f;
%     
%     displacementVectors = [dy, dx];
end