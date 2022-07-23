% ---------------LOCALIZIATION CODE---------------------


rosshutdown
clear all
close all
setenv('ROS_MASTER_URI','http://jawad1-VirtualBox:11311')
rosinit
%subscribe
robotposeodom=rossubscriber('/husky_velocity_controller/odom');
robotposeworld=rossubscriber('/gazebo/link_states');
robotlidar=rossubscriber('/scan');
pause(5)

%declare variables
%alphas cloudold and odom old will be used as inputs to cloud function that
%moves particles fro cloudold to cloud new 
alphas=[0.1,0.5,0.4,0.1];
cloudold=zeros(40,3);
odomold=[0,0,0];
realx=[];
realy=[];
q=0;
%load map
A = imread('MAP1.jpg');%Each pixel is 0.1m
%The imported image is RGB
%%Convert image to B&W
map_bw = im2bw(A,0.9);
%map_bw = 1-map_bw;%line might not be needed, depends 

%figure
%imshow(A)
figure
imshow(map_bw)


particles_probabilities=[];
debug=1;
flag=0;
i=0;
odom_x=[];
odom_y=[];
world_x=[];
world_y=[];
    
size=200;
while flag==0;
    
    %using i to track the loop and helps in debugig    
    i=i+1
    %get world pos
    robotposeworld1=robotposeworld.LatestMessage;
    WorldX = robotposeworld1.Pose(37).Position.X;
    WorldY = robotposeworld1.Pose(37).Position.Y;
    WorldZ = robotposeworld1.Pose(37).Position.Z;
    %put them in vector form
    Worldpose=[WorldX,WorldY,WorldZ];
    %get world orientation in quat
    WorldQuat = [robotposeworld1.Pose(37).Orientation.X,robotposeworld1.Pose(37).Orientation.Y,robotposeworld1.Pose(37).Orientation.Z,robotposeworld1.Pose(37).Orientation.W];
    %change orientation in eularangles
    eulZYX = quat2eul(WorldQuat);
    %chose only orientation along z
    WorldOrientation = eulZYX(3);
    world_x=[world_x WorldX];
    world_y=[world_y WorldY];

    

    %get odom pos
    posodm=robotposeodom.LatestMessage;
    odomx=posodm.Pose.Pose.Position.X;
    odomy=posodm.Pose.Pose.Position.Y;
    %get  orientation in quat
    Quat =[posodm.Pose.Pose.Orientation.X,posodm.Pose.Pose.Orientation.Y,posodm.Pose.Pose.Orientation.Z,posodm.Pose.Pose.Orientation.W];
    %change orientation in eularangles
    eulZYX = quat2eul(Quat);
    %chose only orientation along z
    odomz = eulZYX(3);
    odomnew=[odomx,odomy,odomz];
    
    if abs(sum(odomnew)-sum(odomold))>0.01 % might be changed
        %prediction step
        %start sampling

        cloudnew=cloud(alphas,odomold,odomnew,cloudold);

        cloudold=cloudnew;
        odomold=odomnew;
        q=1;
        %end of sampling part
        %end of prediction step 
       
    else if abs(sum(odomnew)-sum(odomold))<0.0001  && q==1
            
            
            
        figure
        hold on
    
        %plot(odom_x,odom_y,world_x,world_y,'g')
        RGB = [0 128 0]/256;
        scatter(WorldX,WorldY,[],RGB,'filled');
        RGB = [255 0 0]/256;
        scatter(odomx,odomy,[],RGB,'filled');
        RGB = [200 200 100]/256;
        scatter(cloudnew(:,1),cloudnew(:,2),[],RGB);      
        pause(0.05)

        prompt = 'press any key to start correction step ';
        l = input(prompt);
        
        
        % get the sensor reading from our real robot
        %get lidar ranges
        lidarranges=robotlidar.LatestMessage.Ranges;
        lidarrange=lidarranges';
        minang=robotlidar.LatestMessage.AngleMin;
        maxang=robotlidar.LatestMessage.AngleMax;
        incr=robotlidar.LatestMessage.AngleIncrement;
        %get lidar angels
        %lidarang=minang:incr:maxang;
        lidarang=minang:(maxang-minang)/719:maxang;
        lidar_angle=-lidarang;
        
        
        %now we can determine the weight of each particle
        for h=1:1:40 %%%%%%f%%%%%%
            h
        pause(0.05)
        %change to map frame
        lidar_angle_map=lidar_angle + cloudold(h,3);
        %measurment_model_j(cloudold(h,1),cloudold(h,2),lidar_angle_map,lidarrange,debug,map_bw)
        %measurment model
        p=measurment_model_j(cloudold(h,1),cloudold(h,2),lidar_angle_map,lidarrange,debug,map_bw,size);
        particles_probabilities=[particles_probabilities,p]
        end
        
        %now each point has a 
        %resample
        resampled_cloud=resampling(cloudold,particles_probabilities,40);
        
        
        
        RGB = [100 100 100]/256 ;
        scatter(resampled_cloud(:,1),resampled_cloud(:,2),[],RGB);
        hold off
        pause(0.05)
        
        
        cloudold=resampled_cloud;
        particles_probabilities=[];
        q=0;
       
        
        end
    end
    odomold=odomnew;
            
end