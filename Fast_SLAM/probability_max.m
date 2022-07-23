function probability_of_failure_to_get_reading = probability_max(z_from_lidar, z_max)

%{

This function computes the probability that we dont get any reading from
the lidar, because the obstacle is out of range for example 

z_from_lidar (ztk): actual reading from lidar.

z_from_raycast (ztk*): the expected reading from lidar if the robot were at position xt given the map.


%}


if z_from_lidar == z_max
    probability_of_failure_to_get_reading = 1;
else
    probability_of_failure_to_get_reading = 0.01;


end

