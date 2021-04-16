close all
clear
clc

%% Parameters
robot_poses = [0 0 0;
               2 2 -3/4*pi];
maxDistance = 2.5;
resol = 0.36;
field_of_view = 180;
theta_noise = 0.000;
r_noise = 0.000;

x = 1;
y = 1;
theta = 0;
x_assume = 0.85;
y_assume = 0.85;
theta_assume=0;
xmin = -0.5;
xmax = 0.5;
ymin = -0.5;
ymax = 0.5;

obj1 = [-0.2 -0.075;
        0.2 -0.075;
        0.2 0.075;
        -0.2 0.075];
obj2 = [-0.15 -0.1;
        0.15 -0.1;
        0.15 0.1;
        -0.15 0.1];
obj3 = [0 0;
        0.4 0;
        0 0.1];
obj4 = [0 0;
        0.3 0;
        0 0.15];
    
obj1_param = [0 .2;
              pi/2 .0725;
              pi 0.2;
              -pi/2 0.0725];

%% Translate object
translated_object = translateObject(obj1, x, y, theta);
figure(1)
drawObject(translated_object);
hold on

%% Scan object from multiple views
lines = [translated_object([1:end],:), translated_object([2:end,1],:)];
scan_total = [];

for poseIdx = 1:size(robot_poses,1)
    theta = robot_poses(poseIdx,3);
    A = [cos(theta) -sin(theta);
         sin(theta) cos(theta)];
    scan = laserscanHighresol(robot_poses(poseIdx,1), robot_poses(poseIdx,2), robot_poses(poseIdx,3), lines', maxDistance, resol,field_of_view);
    scan_cart = polar2carth(scan);
    scan_world = A*scan_cart + robot_poses(poseIdx,1:2)';
    scan_total(:,size(scan_total,2)+1:size(scan_total,2)+size(scan_world,2)) = scan_world;
end


scan_box_index = scan_total(1,:) <=x+xmax & scan_total(1,:) >=x+xmin & scan_total(2,:) <=y+ymax & scan_total(2,:) >=y+ymin;
scan_box = scan_total(:,scan_box_index);
scatter(scan_total(1,:), scan_total(2,:))

%% Run ransac
ransac_params = [4, 400, 0.01, 5, 3];
box_lines = ransacLines(scan_box, ransac_params);

numlines = size(box_lines,2);
figure(3)
for idx=1:size(box_lines,2)
    plotLine(box_lines(:,idx), 'r', '-')
    hold on
end

drawObject(translated_object);
scatter(scan_box(1,:), scan_box(2,:));

%% Transform object parameter space to world space
[obj_world_alpha, obj_world_r] = lineTransform(obj1_param(:,1), obj1_param(:,2), x_assume, y_assume, theta_assume);
obj_world = [obj_world_alpha, obj_world_r];
for idx=1:size(obj_world,1)
    plotLine(obj_world(idx,:), 'm', '-')
    hold on
end