function probability_of_random_measurement_errors = probability_random(z_from_lidar, z_max)



if z_from_lidar > 0 && z_from_lidar < z_max
    
    probability_of_random_measurement_errors = 1/z_max;
else
    probability_of_random_measurement_errors = 0;

end

