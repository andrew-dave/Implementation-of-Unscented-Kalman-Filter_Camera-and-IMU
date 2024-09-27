 function [covarEst,uEst] = pred_step(uPrev,covarPrev,angVel,acc,dt)
%% BEFORE RUNNING THE CODE CHANGE NAME TO pred_step
    %% Parameter Definition
    % uPrev - is the mean of the prev state
    %covarPrev - covar of the prev state
    %angVel - angular velocity input at the time step
    %acc - acceleration at the timestep
    %dt - difference in time 

%% Computing Sigma Points
%Augmenting uPrev and covarPrev
Q=0.00001;
Qt   = dt*(eye(12) * Q);
uAug = [uPrev; zeros(12,1)];
PAug = [covarPrev, zeros(15,12); zeros(12,15), Qt];
root_PAug = chol(PAug);
n = length(uAug);

%Initialising UKF parameters
alpha = 0.001; % Tuning parameter (controls spread of sigma points)
beta = 2;     % Incorporates prior knowledge of distribution (Gaussian distribution: beta=2)
kappa = 1;    % Secondary scaling parameter

%Calculating Sigma point spread Lambda
lambda = alpha^2 * (n + kappa) - n;

%Compute (2n+1) Sigma points
XAug = zeros(n,2*n+1);
XAug(:,1) = uAug;

%Computing the sigma points for n+1 iterations
for i = 2:n+1
    XAug(:,i) = uAug + sqrt(n + lambda)*root_PAug(:,i-1);
    XAug(:,i+n) = uAug - sqrt(n + lambda)*root_PAug(:,i-1);
end

%% Propogating sigma points through the non-linear function
%State matrix of non-linear function F(x,u,n)
Xt = zeros(15,2*n+1);
for j=1:2*n+1
    % Assign position from augmented state to x1
    x1 = XAug(1:3,j);
    % Assign Orientation(roll,pitch,yaw) from augmented state
    r = XAug(4,j);
    p = XAug(5,j);
    y = XAug(6,j);
    x2 = [r;p;y];
    % Assign Linear velocities from augmented state
    vx = XAug(7,j);
    vy = XAug(8,j);
    vz = XAug(9,j);
    lvel = [vx;vy;vz];
    % Assign gyroscope and accelerometer bias from augmented state
    bg = XAug(10:12,j);
    x4=bg;
    ba = XAug(13:15,j);
    x5=ba;
    %State = [x1; x2; lvel; x4; x5];
    % Assign the noises ng, na, nbg, nba from augmented state
    ng = XAug(16:18,j);
    na = XAug(19:21,j);
    nbg = XAug(22:24,j);
    nba = XAug(25:27,j);
    
    % Calculate inverse of Euler Rate Parameterisation (ZYX) for f(x,u,n)
    G_inv = [(cos(y)*sin(p))/(cos(p)*cos(y)^2 + cos(p)*sin(y)^2), (sin(p)*sin(y))/(cos(p)*cos(y)^2 + cos(p)*sin(y)^2), 1;
                                   -sin(y)/(cos(y)^2 + sin(y)^2),                        cos(y)/(cos(y)^2 + sin(y)^2), 0;
                      cos(y)/(cos(p)*cos(y)^2 + cos(p)*sin(y)^2),          sin(y)/(cos(p)*cos(y)^2 + cos(p)*sin(y)^2), 0];

    % Calculate Rotation matrix (ZYX) for f(x,u,n)
    R=[cos(p)*cos(y),  cos(y)*sin(p)*sin(r) - cos(r)*sin(y),  sin(r)*sin(y) + cos(r)*cos(y)*sin(p)
       cos(p)*sin(y),  cos(r)*cos(y) + sin(p)*sin(r)*sin(y),  cos(r)*sin(p)*sin(y) - cos(y)*sin(r)
             -sin(p),                         cos(p)*sin(r),                         cos(p)*cos(r)];

    % Concatenate all elements of f(x,u,n)
    fxun = vertcat(lvel,G_inv * (angVel - bg),[0;0;-9.8] + R *(acc - ba),nbg,nba);
    
    %Propagating the Sigma Points through the non-linear state function
    Xt(:,j) = [x1 + fxun(1:3)*dt;
        x2 + fxun(4:6)*dt - ng;
        lvel + fxun(7:9)*dt - na;
        x4 + fxun(10:12)
        x5 + fxun(13:15)];
end

%% Calculating uEst and covarEst
%Calculating the Weights for Mean and Covariance
Wm = [lambda / (n + lambda), 0.5 / (n + lambda) + zeros(1, 2 * n)]; % Weights for mean
Wc = [lambda / (n + lambda) + (1 - alpha^2 + beta), 0.5 / (n + lambda) + zeros(1, 2 * n)]; % Weights for covariance

%Initialising Estimated mean state and noise covariance matrix
uEst = zeros(15,1);
covarEst = zeros(15,15);

%Calculating the Weighted mean state Estimate
for k = 1:2*n+1
    uEst = uEst + Wm(k)*Xt(:,k); 
end

%Calculating the Weighted Covariance noise matrix
for l = 1:2*n+1
    covarEst = covarEst + Wc(l)*((Xt(:,l)-uEst) * ((Xt(:,l)-uEst)'));
end

end

