function [laser_distances, imageOut] = laser_scan_model(x, y, min_range, ...
max_range, number_of_rays, min_angle, max_angle, map_image, map_image_rgb, ...
occupied_threshold, distance_step, lidar_rgb, gaussian_power, ...
generate_image, pixelsToAdd)

[map_height, map_width] = size(map_image); 
laser_distances = zeros(1, number_of_rays); 
x_pixels = zeros(1, number_of_rays);
y_pixels = zeros(1, number_of_rays);

    for laserScanIter=1:number_of_rays

        angle = min_angle + laserScanIter * (max_angle - min_angle) ...
            / number_of_rays;
        distance = 1;
        while distance <= max_range

            xMap = max((x) + cos( angle ) * distance,1);
            yMap = max((y) + sin( angle ) * distance,1);

            if (((xMap || yMap) < 1) || xMap > map_width ...
                    || yMap > map_height)
            %We're off the map! Probably need to just quit here 
            % and record max range            
                distance = max_range;
            break;

            end 

            if ( double(map_image(ceil(yMap), ceil(xMap))) ...
                    < occupied_threshold )
            %We've identified an occupied cell, record this 
            %distance and move on
                break;

            end 

            distance = distance + distance_step; 
        end 

        if distance >= max_range
            laser_distances(laserScanIter) = NaN; 
            x_pixels (laserScanIter) =1; 
            y_pixels (laserScanIter) =1; 
        elseif distance <= min_range
            laser_distances(laserScanIter) = NaN;
            x_pixels (laserScanIter) =1; 
            y_pixels (laserScanIter) =1; 
        else 
            laser_distances(laserScanIter) = distance; 
            x_pixels (laserScanIter) = xMap; 
            y_pixels (laserScanIter) = yMap; 
        end  
    end
    
    %Add some gaussian noise so our sensor isn't perfect 
    
    x_pixels=ceil(awgn(x_pixels, gaussian_power)); 
    y_pixels=ceil(awgn(y_pixels, gaussian_power)); 
    laser_distances=awgn(laser_distances, gaussian_power);
    
    if generate_image
        
        [x_pixels,y_pixels] = add_pixels(x_pixels, y_pixels, pixelsToAdd);
        
        imageOut = map_image_rgb; 
        for i = 1 :  map_width
                for j = 1 : map_height
                    if max(i == x_pixels & j == y_pixels) > 0.5
                        imageOut(j,i,:) = cat(3, lidar_rgb(1),  ...
                            lidar_rgb(2), lidar_rgb(3)); 
                    else 
                        imageOut(j,i,:) = map_image_rgb(j,i,:); 
                    end
                end 
        end
    end

end 
