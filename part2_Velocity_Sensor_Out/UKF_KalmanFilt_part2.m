clear; % Clear variables
addpath('../data')
datasetNum = 1; % CH6ANGE THIS VARIABLE TO CHANGE DATASET_NUM
[sampledData, sampledVicon, sampledTime, proj2Data] = init(datasetNum);
% Set initial condition
uPrev = vertcat(sampledVicon(1:9,1),zeros(6,1)); % Copy the Vicon Initial state
covarPrev = 0.01*eye(15); % Covariance constant
savedStates = zeros(15, length(sampledTime)); %Just for saving state his.
prevTime = 0;
vel = proj2Data.linearVel;
angVel2 = proj2Data.angVel;
Z = [vel angVel2];
%% Calculate Kalmann Filter
for i = 1:length(sampledTime)
    %% FILL IN THE FOR LOOP
    if sampledData(i).is_ready == 1
        %time variable
        dt = sampledTime(i) - prevTime;
        prevTime = sampledTime(i); 

        % Function call for the Prediction step of EKF
        [covarEst,uEst] = pred_step(uPrev,covarPrev,sampledData(i).omg,sampledData(i).acc,dt);
        % Function call for the Update step of EKF
        [uCurr,covar_curr] = upd_step(Z(i,:)',covarEst,uEst);
        
        % Update state for the plot
        savedStates(:, i) = uCurr;
        
        % Update uPrev and covarPrev
        uPrev = uCurr;
        covarPrev = covar_curr;
    end
end

plotData(savedStates, sampledTime, sampledVicon, 2, datasetNum);