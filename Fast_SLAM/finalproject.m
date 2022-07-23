
% ---------------------FASTSLAM CODE-------------------------------------------------------------------------------------
%{
In Terminal:
       roslaunch husky_gazebo husky_playpen.launch

In new Terminal:
      rosrun husky_gazebo NEWkey_teleop.py key_vel:=husky_velocity_controller/cmd_vel
%}

rosshutdown
clear all
close all
setenv('ROS_MASTER_URI','http://ubuntu:11311')
rosinit


% Subscribe to get Odometry pose, World pose, and Lidar measurements!
robotposeodom = rossubscriber('/husky_velocity_controller/odom');
robotposeworld = rossubscriber('/gazebo/link_states');
robotlidar = rossubscriber('/scan');
% pause(8)


%% Declare the Size of the Map:
size = 200;

% Now store a number representing each square in the map
for m = 1:size
    for n = 1:size
        row(m,n) = m;
        j(m,n) = n;
    end
end
% Reshape to transform to
row = reshape(row.',1,[]);
j = reshape(j.',1,[]);


%% Declare Variables


% alphas, cloudold, and odomold will be used as inputs to the function "cloud" that
% moves particles from cloudold to cloud new. 
alphas = [0.1,0.5,0.4,0.1];
odomold = [0,0,0];
% Initialize 40 particles
cloudold = zeros(40,3);

realx = [];
realy = [];
q = 0;
particles_probabilities = [];
debug = 0;
flag = 0;
odom_x = [];
odom_y = [];
world_x = [];
world_y = [];


%% create 40 maps with grey scale
for i = 1:40
  maps(:,:,i) = ones(size,size) * 40;
end
%% Perform Maping for the first Time:
% Get Lidar measurements
% Lidar ranges
lidarranges = robotlidar.LatestMessage.Ranges;
lidarrange = lidarranges';
% Lidar Angles:
minang = robotlidar.LatestMessage.AngleMin;
maxang = robotlidar.LatestMessage.AngleMax;
incr = robotlidar.LatestMessage.AngleIncrement;
lidarang = minang:(maxang-minang)/719:maxang;
lidar_angle = - lidarang;

% Start First Mapping
 for w = 1:40
 % Input map should be grey scale.
 fprintf('Updating the Map for Particle = %.4f for the First Time \n', w)
 maps(:,:,w) = updatemap(cloudold(w,1),cloudold(w,2),cloudold(w,3),maps(:,:,w),lidarranges,lidar_angle,size);
 end
 Map=maps(:,:,40);
 figure
 mapflip=flipud(Map);
 imshow(mapflip,[0,100]);
 
%% Initialize Figures
% Cloud figure
f1 = figure('Name','Clouds');
% Map figure
f2 = figure('Name','Maps');

%% Start The LOOP:

while flag==0;
    %% Get Pose from World and Odom
    
    % World pose
    robotposeworld1=robotposeworld.LatestMessage;
    WorldX = robotposeworld1.Pose(37).Position.X;
    WorldY = robotposeworld1.Pose(37).Position.Y;
    WorldZ = robotposeworld1.Pose(37).Position.Z;
    % Put them in vector form
    Worldpose=[WorldX,WorldY,WorldZ];
    % Get world orientation in quat
    WorldQuat = [robotposeworld1.Pose(37).Orientation.X,robotposeworld1.Pose(37).Orientation.Y,robotposeworld1.Pose(37).Orientation.Z,robotposeworld1.Pose(37).Orientation.W];
    % Change orientation in eularangles
    eulZYX = quat2eul(WorldQuat);
    % Choose only orientation along z
    WorldOrientation = eulZYX(3);
    
    % Odom Pose
    posodm = robotposeodom.LatestMessage;
    odomx = posodm.Pose.Pose.Position.X;
    odomy = posodm.Pose.Pose.Position.Y;
    % Get  orientation in quat
    Quat = [posodm.Pose.Pose.Orientation.X,posodm.Pose.Pose.Orientation.Y,posodm.Pose.Pose.Orientation.Z,posodm.Pose.Pose.Orientation.W];
    % Change orientation in eularangles
    eulZYX = quat2eul(Quat);
    % Choose only orientation along z
    odomz = eulZYX(3);
    % Put them in vector form
    odomnew =[odomx,odomy,odomz];
    pause(0.05)
    
    %% Sample as long as the robot is moving, i.e.: Generate the cloud!
    if abs(sum(odomnew)-sum(odomold)) > 0.01
        cloudnew = cloud(alphas,odomold,odomnew,cloudold);  
        cloudold = cloudnew;
        odomold = odomnew;
        q = 1
   
    % When the Robot stops: 
    elseif abs(sum(odomnew)-sum(odomold)) < 0.0001  && q == 1
    %% Draw figures   
    
        figure(f1);
        RGB = [0 125 0]/256;
        scatter(WorldX,WorldY,[],RGB,'filled');
        hold on
        figure(f1);
        RGB = [255 0 0]/256;
        scatter(odomx,odomy,[],RGB,'filled')
        hold on
        figure(f1);
        RGB = [200 200 100]/256;
        scatter(cloudold(:,1),cloudold(:,2),[],RGB); 
        hold on
     %% Prompt the user that the code is ready for the correction step
    
        prompt = 'Press any key to start Correction Step \n';
        l = input(prompt);
        fprintf('Start Correction Step, Please keep your robot static! \n')
      %% Correction step
        
        % Get the Actual Sensor readings from our real robot:)
        % Lidar Ranges
        lidarranges = robotlidar.LatestMessage.Ranges;
        lidarrange = lidarranges';
        minang = robotlidar.LatestMessage.AngleMin;
        maxang = robotlidar.LatestMessage.AngleMax;
        incr = robotlidar.LatestMessage.AngleIncrement;
        
        % Lidar Angels
        lidarang = minang:(maxang-minang)/719:maxang;
        lidar_angle = -lidarang;
        
       %% Get the Weight of each Particle!
        
        for s = 1:40 
       
        % Change Lidar Angle from Robot frame to Map frame
        lidar_angle_map = lidar_angle +  cloudold(s,3);  
        pause(0.05);
        G = maps(:,:,s);
        
        % Apply a Threshold on the map
        % The map should be binary and flipped in the measurement function
        % Change map to binary
        for m=1:size*size
            if G(row(m),j(m)) > 40
                G(row(m),j(m)) = 1;
            else
                G(row(m),j(m)) = 0;
            end
        end
        
        % Flip the map
        map_bw = flipud(G);
        
        % Update the user:
        fprintf('Executing the Measurnment Correction for Particle = %.4f \n', s)
        p=measurment_model_j(cloudold(s,1),cloudold(s,2),lidar_angle_map,lidarrange,debug,map_bw,size);
        
        % Store the Weight
        particles_probabilities = [particles_probabilities,p];
        
        end
        %% Update the Map for each Particle
        
        for w = 1:40
          % Input Map should be grey scale
          fprintf('Updating the Map for Particle = %.4f  \n', w)
          maps(:,:,w) = updatemap(cloudold(w,1),cloudold(w,2),cloudold(w,3),maps(:,:,w),lidarranges,lidar_angle,size);
          
        end
        %% Resampling Step
        % Resample
        [resampled_cloud,resampled_map] = resample_map_cloud(maps,cloudold,particles_probabilities,40);
        
        % Save the Resampled Cloud into the old cloud for the next Time :)   
        cloudold = resampled_cloud;
        % Save the Resampled Map  
        maps = resampled_map;
        particles_probabilities = [];
        q = 0;
        pause(0.05);
        
        %% Show some Maps for some Particles
        figure(f1)
        hold on
        figure(f1);
        RGB = [0 0 0]/256 ;
        scatter(resampled_cloud(:,1),resampled_cloud(:,2),[],RGB,'filled');
        hold on
        
        for q = 35:40

            G = maps(:,:,q);

            % Apply a Threshold on the map
            for m=1:size*size
                if G(row(m),j(m)) > 40
                    G(row(m),j(m)) = 1;
                else
                    G(row(m),j(m)) = 0;
                end
            end  

             figure(f2);    
             Map = G;
             mapflip = flipud(Map);
             imshow(mapflip);
             pause(1)
         
        end
         
        % Prompt the user that the Correction is Done!
        fprintf('Correction done, You can move your Robot! \n')
        prompt = 'Press any key to go back to Sampling :) \n';
        l = input(prompt);
         
    end
       odomold=odomnew;
       figure(f1);
       cla
end

            



