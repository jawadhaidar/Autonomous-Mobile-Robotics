function short_reading_probability = probability_short(lambda, z_from_lidar, z_from_raycast)

%{

This function computes the probability that something passes in front of
the lidar while taking that reading so it get a short reading.

z_from_lidar (ztk): actual reading from lidar.

z_from_raycast (ztk*): the expected reading from lidar if the robot were at position xt given the map.


%}


num = 1;
deno = 1 - exp(-1 * lambda * z_from_lidar);

normalizer = num / deno;

if z_from_lidar > 0 && z_from_lidar < z_from_raycast
    short_reading_probability = normalizer * lambda * exp(-1 * lambda * z_from_lidar);
else 
    short_reading_probability = 0.01;
end

