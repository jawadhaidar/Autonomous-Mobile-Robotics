function measurement_probability = probability_hit(lambda,sigma, z_max, z_from_lidar, z_from_raycast)
%{

This function computes the probability that a measurement is actually done (ie it detects an obstacle).

z_from_lidar (ztk): actual reading from lidar.

z_from_raycast (ztk*): the expected reading from lidar if the robot were at position xt given the map.


%}

    
num = exp( (z_from_lidar - z_from_raycast)^2 / (-2 * sigma^2) );
deno = sqrt(2 * pi * sigma^2 );
N = num/deno;


num = 1;
deno = 1 - exp(-1 * lambda * z_from_lidar);
normalizer = num / deno;

if z_from_lidar > 0 && z_from_lidar < z_max
    measurement_probability = normalizer * N;
else 
    measurement_probability = 0.01;
end
end

