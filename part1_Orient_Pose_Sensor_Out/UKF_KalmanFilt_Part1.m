clear; % Clear variables
addpath('../data')
datasetNum = 1; % CHANGE THIS VARIABLE TO CHANGE DATASET_NUM
[sampledData, sampledVicon, sampledTime,proj2Data] = init(datasetNum);


% Set initial condition
uPrev = vertcat(sampledVicon(1:9,1),zeros(6,1)); % Copy the Vicon Initial state
covarPrev = 0.1*eye(15); % Covariance constant
savedStates = zeros(15, length(sampledTime)); %Just for saving state his.
prevTime = 0; %last time step in real time
pos = proj2Data.position;
pose = proj2Data.angle;
Z = [pos pose];
for i = 1:length(sampledTime)
    %% Fill in the FOR LOOP
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
plotData(savedStates, sampledTime, sampledVicon, 1, datasetNum);