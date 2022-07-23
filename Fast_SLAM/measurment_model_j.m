function total_probability_of_robot_being_at_one_point=measurment_model_j(xp,yp,lidar_angle_map,lidarrange,debug,map_bw,size)
%this functions takes the pos of the particle and calculates the
%measurments that this pos should measure (ztk*, size equal to 20 in this case)
%.And also it takes the real measurments (ztk vector, size equal to lidar rays) as
%input
%lidar_angle_map is the lidar angle wrt map frame

box_length=0.1; %in meters
%weights values
w_hit=0.95;
w_short=0.01;
w_max=0.02;
w_rand=0.02; 
%lamda
lambda = 0.3;
sigma = 1;
z_max=30;

meter=(size/(2))*0.1;
%set raycast function variables
% xc,yc sould be in pixels and wrt map frame 
%( change to map frame by addin 10 (depend on map) meters then change xc from meter to pixel by dividing by box length)
xc=(xp+meter)/box_length; 
yc=(yp+meter)/box_length;
%lidarMin and lidarrange in pixels
%starting theta in degrees
%thetastart in degree

%number of rays
n=20;
lidarmin=10; %in pixels
lidarmax=z_max/box_length;  %interms of pixels
%start at the same angle that the lidar starts and change to degrees
lidar_angle_map=(lidar_angle_map*180)/pi;
thetastart=lidar_angle_map(1); % 1 since first lidar beam is at one
%raycast
fprintf('start ray cast \n')
[z_star_angles,z_star]=castrays(xc,yc,map_bw,n,lidarmin,lidarmax,thetastart,debug);
fprintf('end ray cast \n')

%For each zstar, find the actual sensor reading index corresponding to its z_measurment!
%index=find(z_start_angle==lidar_angle)
%you should try to make it more time eficient
% index=[];
% z_meas=[];
% for i=1:1:n
%     
%     for j=1:1:720
%         if z_star_angles(i)<=lidar_angle(j)+0.1875 && z_star_angles(i)>=lidar_angle(j)-0.1875
%             i
%             
%             index=[index,j];
%             z_meas=[z_meas,lidarrange(j)];
%         end
%     end
% end
%the 2o will always meet the lidar beams at these angles respectivly
index=1:36:36*20;

%z_meas=lidarrange(index);
%Calculate the probability for each reading for the specific possible position
multiplication=1;
fprintf('calculating the probability of the particle \n')
    for r=1:n  %index for relevant beams 
       
        z_meas=lidarrange;
        z_from_lidar = z_meas(index(r));
        if z_from_lidar==Inf
           z_from_lidar=z_max;
        end
        z_from_raycast = z_star(r);

        weighted_hit = 10* w_hit * probability_hit(lambda,sigma, z_max, z_from_lidar, z_from_raycast);
        weighted_short = 10* w_short * probability_short(lambda, z_from_lidar, z_from_raycast);
        weighted_max = 10 * w_max * probability_max(z_from_lidar, z_max);
        weighted_random = 10* w_rand * probability_random(z_from_lidar, z_max);


        probability_of_one_beam = weighted_hit + weighted_short + weighted_max + weighted_random;
        %pause(0.5);
        multiplication = probability_of_one_beam * multiplication;

         
 
    end
 total_probability_of_robot_being_at_one_point = multiplication

%You should multiply all these probabilities together to get the final probability of ALL these
%readings together if the robot is in that specific position.
end 
