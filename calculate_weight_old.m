
        function obj=calculate_weight(obj, sensor_measurement, low_dist, ...
                high_dist)
            %Let's first take a look and see if we've got a NaN in 
            %one or both sensor locations. If we've got one in both,
            %let's define the distance for that case to be low. 
            %If we've only got one NaN, let's assume distance is high. 
            
            num_points=length(sensor_measurement); 
            
            for i = 1 : num_points
                sensor_nan=isnan(sensor_measurement(i)); 
                particle_nan=isnan(obj.distances(i));
                
                distance_difference=zeros(1, num_points); 
                
                if (sensor_nan | particle_nan) 
                    distance_difference(i) = high_dist; 
                else
                    distance_difference(i) = abs(sensor_measurement(i) - ...
                        obj.distances(i));
                end
                    
            end
           
            obj.weight = sum(distance_difference)...
                /num_points;
            
        end