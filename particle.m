    classdef particle < handle
    properties 
        x = 0; 
        y = 0; 
        weight = 1.0;
        normalized_weight = 1.0; 
        distances = []; 
    end
    
    methods
        % Get some random locations for our particles. We're talking in 
        % pixels here so be sure to round to nearest integer value
        
        function obj=randomize_location(obj,x_size, y_size)
            rand_location = [x_size, y_size].*rand(1,2);
            obj.x=floor(rand_location(1)); 
            obj.y=floor(rand_location(2)); 
        end
        

        function obj=calculate_distances(obj, map_image, min_range, ... 
                max_range, rays, min_angle, max_angle, distance_step, ...
                power)
            
          [obj.distances] = laser_scan_model (obj.x, obj.y, ...
              min_range, max_range, rays, min_angle, max_angle, ...
              map_image, map_image, 0.5, 1,[255,0,0], power, false, 0);  
          
        end
        
        function obj=set_x_y(obj, desiredX, desiredY)
            
          obj.x = floor(desiredX); 
          obj.y = floor(desiredY); 
          
        end
        
        % Assume we've calculated some laser scan distances. We need
        % to come up with a weight now. 
        
        function obj=calculate_weight(obj, sensor_measurement, low_dist, ...
                high_dist)
            %Let's first take a look and see if we've got a NaN in 
            %one or both sensor locations. If we've got one in both,
            %let's define the distance for that case to be low. 
            %If we've only got one NaN, let's assume distance is high. 
            
            num_points=length(sensor_measurement); 
            
            
            sensor_nan=isnan(sensor_measurement); 
            particle_nan=isnan(obj.distances);
            
            sensor_measurement(sensor_nan) = high_dist; 
            obj.distances(particle_nan) = high_dist; 

            p_i=norm(obj.distances-sensor_measurement); 

             
            obj.weight = p_i; 
            
        end
        
        % Once we've got some weights we need to normalize these against
        % our higest weight. 
        
        function obj=normalize_weight(obj,max_weight)
            obj.normalized_weight=obj.weight/max_weight;    
        end
        
        
    end
    
end

