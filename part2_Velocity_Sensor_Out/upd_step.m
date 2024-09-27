function [uCurr,covar_curr] = upd_step(z_t,covarEst,uEst)
%% BEFORE RUNNING THE CODE CHANGE NAME TO upd_step
%% Parameter Definition
%z_t - is the sensor data at the time step
%covarEst - estimated covar of the  state
%uEst - estimated mean of the state

%% Calculate Sigma points
%Initialise variables for computation of sigma points
alpha = 0.001; % Tuning parameter (controls spread of sigma points)
beta = 2;     % Incorporates prior knowledge of distribution (Gaussian distribution: beta=2)
kappa = 1;    % Secondary scaling parameter

% Finding size of state vector
n = length(uEst);
%Calculating Sigma point spread Lambda
lambda = alpha^2 * (n + kappa) - n;

%Compute (2n+1) Sigma points
Xt = zeros(n,2*n+1);
Xt(:,1) = uEst;

%Compute sqrt of covarEst using Cholesky Decomposition
root_Covar = chol(covarEst,"lower");
%Computing the sigma points for n+1 iterations
for i = 2:n+1
    Xt(:,i) = uEst + sqrt(n + lambda)*root_Covar(:,i-1);
    Xt(:,i+n) = uEst - sqrt(n + lambda)*root_Covar(:,i-1);
end

%% Initialise the Rotations and translations
% Camera to body homogenous transformation Matrix
H_C2B = [0.7071, -0.7071, 0, 0.0283; -0.7071, -0.7071, 0, -0.0283; 0, 0, -1, -0.03; 0, 0, 0, 1];
% Extract camera to body rotation matrix
R_C2B = H_C2B(1:3,1:3);
% Body to camera Rotation matrix
R_B2C = R_C2B';
% Initialise the skew symmetric matric for the camera to body translation
T_C2B = H_C2B(1:3,4);
SkewT_C2B = [0, -T_C2B(3), T_C2B(2); T_C2B(3), 0, -T_C2B(1); -T_C2B(2), T_C2B(1), 0];
% Body to World Rotation matrix
R_B2W = eul2rotm(uEst(4:6)');
% World to Body Rotation matrix
R_W2B = inv(R_B2W);

%% Propogate sigma points through Zt
% Initialise the additive noise
vt = normrnd(0,(0.0001),[3,1]);
Z_t = zeros(3, ((2*n)+1));
% Noise matrix R for introducing Sensor noise 
R = eye(3) * 0.0001;
% Find all update vectors through iteration
for i = 1:((2*n)+1)
    x3 = Xt(7:9,i);
    % Non-linear update
    Z_t(:,i) = (R_B2C * R_W2B*x3) - (R_B2C * SkewT_C2B * (R_C2B * z_t(4:6,1))) + vt;
end

%% Computing Mean and Covariance Matrices
%Calculating the Weights for Mean and Covariance
Wm = [lambda / (n + lambda), 0.5 / (n + lambda) + zeros(1, 2 * n)]; % Weights for mean
Wc = [lambda / (n + lambda) + (1 - alpha^2 + beta), 0.5 / (n + lambda) + zeros(1, 2 * n)]; % Weights for covariance

% Initialise measurement update
Zu_t = zeros(3,1);
% Computing weighted sum of update
for i = 1:((2*n)+1)
    Zu_t = Zu_t + (Wm(i) * Z_t(:,i));
end

% Computing weighted sum of predicted covariance and cross covariance
C_t = zeros(15,3);
S_t = zeros(3,3);
for i = 1:((2*n)+1)
    C_t = C_t + (Wc(i) * (Xt(:,i) - uEst) * (Z_t(:,i) - Zu_t)');
    S_t = S_t + (Wc(i) * (Z_t(:,i) - Zu_t) * (Z_t(:,i) - Zu_t)');
end
S_t = S_t + R;

%% Computing uCurr and covarCurr
 % Compute Kalman Gain
    KalG = C_t / S_t;
    % Compute current mean
    uCurr = uEst + KalG * (z_t(1:3) - Zu_t);
    % Compute current covariance
    covar_curr = covarEst - (KalG * S_t * KalG');
end

