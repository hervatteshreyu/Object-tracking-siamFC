%--------------------------------------------------------------------------
%% Kalman Filter
%--------------------------------------------------------------------------
%  
% This function implements a linear Kalman filter.
%
% [in] : [A, B, u, H, P, R, Q, x] (kalman parameters)
% [in] : input (tracked ROI)
%
% [out] : kalmanOutput (corrected ROI) 
% [out] : [P, x] (Updated Kalman parameters)
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author : Skanda Bharadwaj 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%
function [kalmanOutput, P, x] = kalmanFilter(A, B, u, H, P, R, Q, x, input)

    % Estimate the next state
    x = A*x + B*u;
    
    % Estimate the error covariance
    P = A*P*A' + Q;
    
    % Kalman Gain Calculations
    K = P*H'*pinv(H*P*H'+R);
    
    % Update the estimation
    if(~isempty(input)) %Check if we have an input
        x = x + K*(input - H*x);
    end
    
    % Update the error covariance
    P = (eye(size(P,1)) - K*H)*P;
    
    % Save the measurements for plotting
    kalmanOutput = H*x;

end

%--------------------------------------------------------------------------
%% END






