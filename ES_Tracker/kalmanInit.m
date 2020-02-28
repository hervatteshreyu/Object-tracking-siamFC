%--------------------------------------------------------------------------
%% Initialization of Kalman Filter
%--------------------------------------------------------------------------
%  
% This function call initializes the kalman filter parameters such as state
% matrix, control matrix, sampling rate, measurement matrix, errors,
% covariance matric etc. 
%
% [in] : Height, Width (image height and width)
%
% [out] : [A, B, u, H, P, R, Q, x] -> initilized Kalman parameters
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author : Skanda Bharadwaj 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%
function [A, B, u, H, P, R, Q, x] = kalmanInit(ROI)
    
    % smapling rate
    dt=0.05;

    % State matrix (NxN) N -> Number of states 
    A = [1 0 dt  0;
         0 1  0 dt;
         0 0  1  0;
         0 0  0  1;
        ];

    % Control matrix (Nx1)
    B = [(dt^2)/2;
         (dt^2)/2;
         dt;
         dt;
        ];

    % Control vector (acceleration).
    u = 0.002;

    % Measurement Matrix
    H = [1 0 0 0;
        0 1 0 0];

    % Uncertainty
    State_Uncertainty = 0.5;
    P = State_Uncertainty * eye(size(A,1));


    Meas_Unertainty = 10;
    R = Meas_Unertainty * eye(size(H,1));

    % Covariance Matrix
    Q = [(dt^2)/4     0    (dt^3)/2     0    ;
            0     (dt^2)/4     0    (dt^3)/2 ;
         (dt^3/2)     0     (dt^2)      0    ;
            0     (dt^3)/2     0     (dt^2)  ;
        ];
    % Initial guess of the state values
    x = [ROI(1); ROI(2); 0; 0];
end

%--------------------------------------------------------------------------
%% END