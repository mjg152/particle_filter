clear; 
close all; 

%% Algorithm Inputs

number_particles = 100; 
nan_distance_low= 20; %distance to use for weight calculation if both the 
%sensor and the particle read nan for a given lidar range 
nan_distance_high=150; %distance to use for weight calculation if only one
%the sensor or the particle read NaN. Should be viewed as a penalty 
%weighting factor for this situation. 
create_plots = false; %set to true to store plots 
min_lidar_dist = 1; %Minimum lidar distance 
max_lidar_dist = 100; % Max lidar distance 
num_rays = 36; % Max lidar distance 
lidar_min_angle = 0; % Max lidar distance 
lidar_max_angle = 2*pi; % Max lidar distance 
distance_step = 1; % Number of pixels to move in each iteration 
lidar_rgb = [255, 128, 0]; % RGB color code for lidar pings 
gaussian_power_robot = 1; %Decibel ratings for gaussian noise applied to lidar signals
%Higher gaussian power indicates less noise 
gaussian_power_particles = 1000; %We may select a different Gaussian power
%for particle lidar calculations. 
lidar_pixels_to_add = 2; %increase pixel count of lidar points for 
%visualization purposes. 
resampling_ratio = 0.4; %of particle array components to be kept 
random_walk =20; %random walk pixels 
recently_moved_random_walk =100; %random walk pixels 
random_walk_cycles = 6; %particle filter cycle increment for random walk  


%% Set up our map 
images_directory = 'C:\Users\michael\OneDrive\Documents\Case Western\EECS 499 Algorithmic Robotics\Particle Filter Algorithm\Images\'
simple_rooms = 'simple_rooms.png'; 

image_rgb=imread([images_directory, simple_rooms]); 
image_bw=im2bw(image_rgb); 

[y_pixels, x_pixels]=size(image_bw); 

%% Initialize algorithm 

for i = 1: number_particles
     p = particle;
     particleArray(i)=p.randomize_location(x_pixels, y_pixels)
end

% Let's make an image of our original randomized particle locations
% We'll display the particles as if they're larger than they are for our
% viewing pleasure 

imageOut = plot_on_image(image_rgb, [particleArray.x], [particleArray.y], ...
    75, 0 , 130, 2); 
figure; 

imshow(imageOut); 
title('Map with Random Particle Distribution, Particles in Purple', 'FontSize', 26); 

set(gcf, 'Position', get(0,'Screensize'))

if create_plots 
    saveas(gcf, 'InitialRandomDistribution', 'epsc') 
end; 

%Let the user select the original location of the robot 
[robot_x,robot_y] = ginput(1); 
close all;

%These are pixel locations, don't need to deal with Matlab interpolation 
robot_x = ceil(robot_x); 
robot_y = ceil(robot_y); 

% Now let's show the robot on the image in red, this time really large. 
originalRobot_OriginalDistribution=plot_on_image(imageOut, robot_x, ...
    robot_y, 255, 0, 0, 4); 
imageWithRobot=plot_on_image(image_rgb, robot_x, ...
    robot_y, 255, 0, 0, 4); 


close all; 
fig=figure; 
h=imshow(originalRobot_OriginalDistribution) 
set(h,'ButtonDownFcn',{@get_last_click});
hold on; 

title('Map with Random Particle Distribution and Robot in Red', 'FontSize', 26); 

set(gcf, 'Position', get(0,'Screensize'))

if create_plots 
    saveas(gcf, 'OriginalRobotOriginalDistribution', 'epsc') 
end; 



counter = 0; 
buttonUpdateCounter = 0; 
initPlotsCreated=false; 
recentlyMoved=false; 

while true
    
      
    if exist('new_x_pos') %Then someone clicked on the map and moved
        % the robot
        xMove=new_x_pos-robot_x; 
        yMove=new_y_pos-robot_y;  
        
        robot_x = new_x_pos; 
        robot_y = new_y_pos; 
        
        %Let's move the particles by a similar amount, but add in some gaussian
        %noise to simulate the error in our estimate
    
        if xMove > 1 | yMove > 1

            xMove=awgn(ones(1, number_particles)*xMove, 1); 
            yMove=awgn(ones(1, number_particles)*yMove, 1); 

            for i = 1 : number_particles
                particleArray(i).x = ...
                    floor(min(max(particleArray(i).x + xMove(i),1), x_pixels)); 
                particleArray(i).y = ...
                    floor(min(max(particleArray(i).y + yMove(i),1), y_pixels)); 
            end

            %Update our picture of the robot         
            imageWithRobot=plot_on_image(image_rgb, robot_x, ...
            robot_y, 255, 0, 0, 4); 

            buttonUpdateCounter=buttonUpdateCounter+1; 
            recentlyMoved=true;

        end
        
    end 
    
    % Now time for some initial calcs based on our robot location 
    [laser_distances, part_robot_lidar] = laser_scan_model ( robot_x, robot_y, ...
        min_lidar_dist, max_lidar_dist, num_rays, lidar_min_angle, ...
        lidar_max_angle, image_bw, originalRobot_OriginalDistribution,  ...
        0.5, distance_step, lidar_rgb, 1, true, 2);

    if ~initPlotsCreated
        fig1=figure; 
        imshow(part_robot_lidar);
        title('Robot (Red), Particles (Purple), Robot Lidar Pings (Orange)', 'FontSize', 26);  
        set(gcf, 'Position', get(0,'Screensize'))

        if create_plots 
            saveas(gcf, 'RobotParticlesLidar', 'epsc') 
        end;
        
        close(fig1); 
        initPlotsCreated = true; 
    end 
    
    %Sum up our previous weights 
    
    % Generate particle LIDAR pings and calculate weights 
    for i = 1 : number_particles

        %First let's generate some lidar pings for our particles

        particleArray(i).calculate_distances(image_bw, min_lidar_dist, ...
            max_lidar_dist, num_rays, lidar_min_angle, lidar_max_angle, ...
            distance_step, gaussian_power_particles); 

        %Now let's calculate the weight based on the difference between
        %these pings and the laser_scan distances 

        particleArray(i).calculate_weight(laser_distances,nan_distance_low, ...
        nan_distance_high); 


    end 
    
    weights=[particleArray.weight];
    maxWeights = max(weights); 

    %Scale the weights by our weight sum 
    for i = 1 : number_particles
        particleArray(i).normalize_weight(maxWeights); 
    end 
    
    %Resample, pick 100 particles randomly from the group with the lowest 
    %normalized weights. Start by dropping some of our worst particles 

    normalizedWeights = [particleArray.normalized_weight];
    [sortedWeights, indices] = sort(normalizedWeights, 'ascend'); 
    maxIndex = ceil(length(sortedWeights)*resampling_ratio); 
    indicesToKeep = indices(1:1:maxIndex); 
        
    isolatedParticleArray=particleArray(indicesToKeep);
    isolatedParticleArrayLength = length(isolatedParticleArray); 

    %Now pack newParticleArray in a loop by picking from our 
    %isolatedParticleArray at random just keep our 25 best though
    for i = 1 : number_particles
        if i > 25
            indexToSelect = ...
            min(max(ceil(rand(1)*isolatedParticleArrayLength),1)...
            ,isolatedParticleArrayLength) ; 
            newParticleArray(i) = isolatedParticleArray(indexToSelect); 
        else
            newParticleArray(i) =  isolatedParticleArray(i); 
        
        end 
    end 
    clear particleArray; 
    particleArray=newParticleArray; 
    clear newParticleArray; 

    %And our new particles 
    imageWithRobotParticles=plot_on_image(imageWithRobot, [particleArray.x], ...
        [particleArray.y], 75, 0 , 130, 2);
    
    weights=[particleArray.normalized_weight];
    particleXs=[particleArray.x];
    particleYs=[particleArray.y];
    maxWeight = max(weights); 
    minWeight = min(weights); 
    totalWeight = sum(weights);
    
    estimateX=sum(particleXs)/length(particleXs); 
    estimateY=sum(particleYs)/length(particleYs); 
    estimateCircle = int32([estimateX estimateY 6]);
       
    shapeInserter = vision.ShapeInserter('Shape','Circles', ...
        'BorderColor','Custom', 'CustomBorderColor',uint8([255 255 0]));
    
    imageWithRobotParticlesCircle= ...
        shapeInserter(imageWithRobotParticles, estimateCircle);
    
    weightedX=sum(particleXs.*(weights))/length(particleXs); 
    weightedY=sum(particleYs.*(weights))/length(particleYs); 
    weightCircle = int32([weightedX weightedY 6]);
    
    shapeInserter = vision.ShapeInserter('Shape','Circles', ...
        'BorderColor','Custom', 'CustomBorderColor',uint8([255 255 0]));
    
    imageWithRobotParticlesCircle= ...
        shapeInserter(imageWithRobotParticlesCircle, estimateCircle);
    
    % [x1 y1 radius1;x2 y2 radius2]
    
    figure(fig); 
    h=imshow(imageWithRobotParticlesCircle);
    set(h,'ButtonDownFcn',{@get_last_click});
    %set(gcf, 'Position', get(0,'Screensize'))
    
    counter=counter + 1; 
    
    %Random walk 
    if counter >= random_walk_cycles
        
        if recentlyMoved
            walk_x=(recently_moved_random_walk*2*rand(1,number_particles))...
                -recently_moved_random_walk; 
            walk_y=(recently_moved_random_walk*2*rand(1,number_particles))...
                -recently_moved_random_walk; 
            for i = 1: number_particles 
                if i ~= indices(1:50) % keep our 50 best particles 
                    p=particle;
                    desiredX=max(min(particleArray(i).x+walk_x(i), x_pixels),1); 
                    desiredY=max(min(particleArray(i).y+walk_y(i), y_pixels),1);

                    randWalkPartArray(i)=p.set_x_y(desiredX, desiredY);
                    clear p; 

                end

            end
            recentlyMoved=false;
        else
            walk_x=(random_walk*2*rand(1,number_particles))-random_walk; 
            walk_y=(random_walk*2*rand(1,number_particles))-random_walk; 
            for i = 1: number_particles 
                if i ~= indices(1:50) % keep our 50 best particles 
                    p=particle; 
                    desiredX=max(min(particleArray(i).x+walk_x(i), x_pixels),1); 
                    desiredY=max(min(particleArray(i).y+walk_y(i), y_pixels),1);

                    randWalkPartArray(i)=p.set_x_y(desiredX, desiredY);
                    clear p; 

                end

            end
        end
        clear particleArray; 
        particleArray = randWalkPartArray;
        clear randWalkPartArray; 
        
        counter = 0; 
        
        imageWithRobotParticles=plot_on_image(imageWithRobot, [particleArray.x], ...
        [particleArray.y], 75, 0 , 130, 2);
    
        figure(fig); 
        h=imshow(imageWithRobotParticles);
        set(h,'ButtonDownFcn',{@get_last_click});
        %set(gcf, 'Position', get(0,'Screensize'))
        
        pause(0.1); 
    end 
    
    pause(0.1);
end