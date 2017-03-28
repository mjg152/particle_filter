test = particle;
x_diff=-10; 
y_diff=10;

test.set_x_y(robot_x+x_diff, robot_y+y_diff);

test.calculate_distances(image_bw, min_lidar_dist, ...
            max_lidar_dist, num_rays, lidar_min_angle, lidar_max_angle, ...
            distance_step, gaussian_power_particles); 
        
particle_distances = test.distances; 
particle_distances(isnan(particle_distances)) = 150; 
laser_distances(isnan(laser_distances)) = 150; 

norm(particle_distances - laser_distances)