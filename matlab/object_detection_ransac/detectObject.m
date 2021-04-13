close all
clear
clc

x = 0;
y = 0;
theta = 0;
maxDistance = 2.5;
resol = 0.36;
field_of_view = 180;
theta_noise = 0.000;
r_noise = 0.01;

xmin = 0.5;
xmax = 1.5;
ymin = 0.5;
ymax = 1.5;

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
        0 0.5];
    
% drawObject(obj4);
% hold on

translated_object = translateObject(obj4, 1, 0.8, 0);
figure(1)
drawObject(translated_object);
hold on

%% Generate lines and perform scan
lines = [translated_object([1:end],:), translated_object([2:end,1],:)];

scan = laserscanHighresol(x, y, theta, lines', maxDistance, resol,field_of_view);
r_noise_samples = normrnd(0,r_noise,1,size(scan,2));
theta_noise_samples = normrnd(0,theta_noise,1,size(scan,2));
scan_noisy = scan + [theta_noise_samples;r_noise_samples];
figure
polarplot(scan_noisy(1,:), scan_noisy(2,:))
%% Convert to polar coordinates and filter
cart = polar2carth(scan_noisy);
index = cart(1,:) <=xmax & cart(1,:) >= xmin & cart(2,:) <=ymax & cart(2,:) >=ymin;
cart_box = cart(:, index);

figure(1)
scatter(cart_box(1,:), cart_box(2,:));
axis([0.5 1.5 0.5 1.5]);

%% Run ransac
ransac_params = [2, 40, 0.05, 3, 3];
box_lines = ransacLines(cart_box, ransac_params);
figure(3)
for idx=1:size(box_lines,2)
    plotLine(box_lines(:,idx), 'r', '-')
    hold on
end

drawObject(translated_object);
scatter(cart_box(1,:), cart_box(2,:));
axis([0.5 1.5 0.5 1.5]);

%% Get intersection point of lines
A = [cos(box_lines(1,1)) sin(box_lines(1,1));
     cos(box_lines(1,2)) sin(box_lines(1,2))];
b = box_lines(2,:)';
intersect = A\b;

%% Get side lengths
c1 = cart_box(1,:);
c2 = cart_box(1,:);