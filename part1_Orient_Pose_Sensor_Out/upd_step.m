function [uCurr,covar_curr] = upd_step(z_t,covarEst,uEst)
%% BEFORE RUNNING THE CODE CHANGE NAME TO upd_step
%% Parameter Definition
%z_t - is the sensor data at the time step
%covarEst - estimated covar of the  state
%uEst - estimated mean of the state

%Initializing the C matrix for linearization and is not computed since the
%measurement model z_t is available
C = horzcat(eye(6), zeros(6,9)); %Dimension 6x15
% 6x6 matrix R for introducing Sensor noise for linear update
R = eye(6) * 0.00001;
% Kalman Gain Calculation
KalG = (covarEst * C') / ((C * covarEst * C') + R);
% Current Covariance matrix
covar_curr = covarEst - (KalG * C * covarEst);
% Current State
uCurr = uEst + (KalG * (z_t - (C*uEst)));

end
