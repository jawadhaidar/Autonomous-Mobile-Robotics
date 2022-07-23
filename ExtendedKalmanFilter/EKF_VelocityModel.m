
%% Extended Kalman Filter Algorithm
clc
clear all


%%
% ---------------------------------Setting up the environment---------------------------------------

% Initial Covariance, i.e. initial uncertainty in Motion Model
Covar=[0.01,0,0;0,0.01,0;0,0,0.01];

v = 0.1;
w = 0.2;
theta = 0;

mean = [0,0,0]';
TimeStamps = 1;

%% Map

% First, we have to assume a couple of things: Let's assume we only have
% one Landmark:
% Define how many Landmarks we have, assume we have one:
Landmarks = 1;

fprintf("EKF implementation for Only One Iteration :) \n")
fprintf("The number of Landmarks is also set in the code as %.4f \n", Landmarks)

% We know the position of all these landmarks (They are the map). bcz they
% are not given in the question, I will randomly set them here. Each
% landmark has an x y and z, assume they are normally distributed around a
% mean = 0 and std for each. 
std_x = 0.1;
std_y = 0.1;
std_z = 0.1;
m = zeros (Landmarks, 3);

for i = 1:Landmarks
    
 % The x, y, and z for each Landmark will be included in the m matrix 
 % representing the map 
 m(i,:) = [std_x * randn; std_y * rand; std_z * rand];
 
 
end
    

%% Uncertainties

% Now m contains our map, i.e. the position x y and z of each Landmark!
% Uncertainty in the measurement model is a scalar, because our measurement
% model only consists of the range reading:

 sigma_r = 0.1;
 Q_t = sigma_r^2;
 
 % Identity Matrix
 I = [1,0,0;0,1,0;0,0,1];
  
 % In order to get the estimated covariance later in the prediction step,
 %  we need the Jacobian G of the Motion model. Because x, y,
 % and z are linearly independent, G reduces to an identity Matrix of 3x3
 
 delta_t = 1;   
 r1 = -(v/w) * cos(theta) + (v/w) * cos(theta + w * delta_t);
 r2 = -(v/w) * sin(theta) + (v/w) * sin(theta + w * delta_t);
 
 G = [1, 0, r1; 0, 1, r2; 0, 0, 1];
 
 r3 = - sin(theta) + sin(theta + w * delta_t);
 r4 = (v * (sin(theta) - sin(theta + w * delta_t))/w^2) + (v * cos(theta+w*delta_t)*delta_t)/w;
 r5 = (cos(theta) - cos(theta + w *delta_t))/w;
 r6 = -(v * (cos(theta) - cos(theta + w * delta_t))/w^2) + (v * sin(theta+w*delta_t)*delta_t)/w;
 V = [r3,r4;r5,r6;0,delta_t];
 

 % The noise in the measurement model is mainly caused by the uncertainty in
 % velocity 
 % Assign standard deviation of velocity
 % Uncertainty in Motion Model R_t due to Velocity:
 
alfa1 = 0.1;
alfa2 = 0.1;
alfa3 = 0.1;
alfa4 = 0.1;

M = [alfa1*v^2 + alfa2*w^2, 0; 0, alfa3*v^2 + alfa4*w^2];
 
 %% Prediction Step:
      
    for t = 1: TimeStamps

  
    %  Assume Robot can get odometry measurement every 0.5s

    b = [((-v/w) * sin(theta) + (v/w) * sin(theta + w*delta_t)), ((v/w) * cos(theta) - (v/w) * cos(theta + w*delta_t)), w*delta_t];
    
    mean = mean + b;
    
    fprintf("------------------------\n");
    fprintf("Prediction Step \n\n")
    fprintf("The Estimated Mean  x  is: %.4f \n", mean(1))
    fprintf("The Estimated Mean  y  is: %.4f \n", mean(1))
    fprintf("The Estimated Mean  z  is: %.4f \n", mean(1))



    % Now we have the estimated mean of the State, let's calculate the estimated Covariance:
    % Now we are ready to calculate our Estimated Covariance, but first , we
    % need to have the Covariance at t=0, let's assume it:
    Covar = G * Covar * G'+ V * M * V';
    fprintf("The Estimated Covariance is:\n")
    Covar
    fprintf("\n\n")
    end
    
    
    %% Correction 
        %-----------------------------CorrectionStep:---------------------------

        % The  correction Step is based on the measurement from the acoustic
        % signals coming from the beacons. In this exercise, the measurement is
        % only the Range r, i.e. z is only the distance ("The robot also knows the 
        % location of all beacons"), and no angle is included: ("However, the robot 
        % cannot sense the direction from which it received a signal")
        
        for i = 1:Landmarks
        % Now Calculate q, the estimatd distance between our robot and the Landmark
         q = (m(i,1) - mean(1))^2 + (m(i,2) - mean(2))^2 + (m(i,3) - mean(3))^2;

        % Get the Measurement z, which is the sqrt of q!
         z_estimated = sqrt(q);

         % The reading of each Landmark at t; Assign them randomly with
         % mean=z_estimated and std = 0.1. However, In real life, We get z_actual
         % DIRECTLY from our sensor. This is just an assumption here bcz we dont
         % have a physical sensor.

         z_actual = z_estimated + 0.1 * randn;

         % Get the Jacobian H of the measurement Model
         H = [(mean(1)- m(1,1))/z_estimated, (mean(2)- m(1,2))/z_estimated, (mean(3)- m(1,3))/z_estimated];

         % Finally, we can calculate our Kalman Gain:
         K = Covar * H' * inv(H * Covar * H' + Q_t);

         % Correction Step ladies and Gents:

         mean_corrected = mean + K * (z_actual - z_estimated);
         Covar_corrected = (I - K*H)* Covar;

         % We just got the corrected mean and Covariance
         % using only One Landmark!
         mean = mean_corrected;
         Covar = Covar_corrected;
        end
        fprintf("------------------------\n");
        fprintf("Correction Step \n\n")
        fprintf("The Corrected Mean  x  is: %.4f \n", mean(1))
        fprintf("The Corrected Mean  y  is: %.4f \n", mean(2))
        fprintf("The Corrected Mean  z  is: %.4f \n", mean(3))
        
        fprintf("The Corrected Covariance is:\n")
        Covar

