clear;
close all; 

nan_distance_low= 20; %distance to use for weight calculation if both the 
%sensor and the particle read nan for a given lidar range 
nan_distance_high=150; %distance to use for weight calculation if only one
%the sensor or the particle read NaN. Should be viewed as a penalty 
%weighting factor for this situation. 
min_lidar_dist = 1; %Minimum lidar distance 
max_lidar_dist = 100; % Max lidar distance 
num_rays = 36; % Max lidar distance 
lidar_min_angle = 0; % Max lidar distance 
lidar_max_angle = 2*pi; % Max lidar distance 
distance_step = 1; % Number of pixels to move in each iteration 
lidar_rgb = [255, 128, 0]; % RGB color code for lidar pings 

images_directory = 'C:\Users\michael\OneDrive\Documents\Case Western\EECS 499 Algorithmic Robotics\Particle Filter Algorithm\Images\'
simple_rooms = 'simple_rooms.png'; 

image_rgb=imread([images_directory, simple_rooms]); 
image_bw=im2bw(image_rgb); 

imshow(image_bw) 

robot_x = 62; 
robot_y = 215; 

particle_x = 348; % 283 
particle_y = 145;

particle_x = 72; % 196 
particle_y = 225;

particle_x = 88; % 308 
particle_y = 260;

particle_x = 246; % 318
particle_y = 152;

particle_x = 368; % 356
particle_y = 24;

particle_x = 67; % 185
particle_y = 220; 

[laser_distances] = laser_scan_model ( robot_x, robot_y, ...
min_lidar_dist, max_lidar_dist, num_rays, lidar_min_angle, ...
lidar_max_angle, image_bw, image_bw,  ...
0.5, distance_step, lidar_rgb, 1, false, 2);

[laser_distances_part] = laser_scan_model ( particle_x, particle_y, ...
min_lidar_dist, max_lidar_dist, num_rays, lidar_min_angle, ...
lidar_max_angle, image_bw, image_bw,  ...
0.5, distance_step, lidar_rgb, 1000, false, 2);

num_points=length(laser_distances); 
            
sensor_nan=isnan(laser_distances); 
particle_nan=isnan(laser_distances_part);

laser_distances(sensor_nan) = nan_distance_high; 
laser_distances_part(particle_nan) = nan_distance_high; 

p_i=norm(laser_distances_part-laser_distances)


%Test Distribution 
pd = makedist('Normal' , 'mu', 0, 'sigma', 2)
x=[-5:0.01:5];
pdf_custom = pdf(pd,x);

figure; 
plot(x, pdf_custom)
xlabel('errorMagnitude / 100', 'FontSize', 26); 
ylabel('Probability', 'FontSize', 26); 
