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
r_noise = 0.000;

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
        0 0.15];
    
objLengths = [0.15 0.4;
              0.4 0.15;
              0.2 0.3;
              0.3 0.2;
              0.1 0.4;
              0.15 0.3;
              0.1 norm([0.1 0.4]);
              0.4 norm([0.1 0.4]);
              0.15 norm([0.15 0.3]);
              0.3 norm([0.15 0.3])];
objIds = [1 1 2 2 3 4 3 3 4 4];
angleOffsets = [-pi/2 -pi/2 -pi/2 -pi/2 -pi/2 -pi/2 pi-atan(.4/.1) pi/2+atan(.1/.4) pi-atan(.3/.15) pi/2+atan(.15/.3)];
positionOffsets = [-0.2 0.0725;
                   0.2 0.0725;
                   -0.15 0.1;
                   0.15 0.1;
                   0 0;
                   0 0;
                   0 -0.1;
                   -0.4 0;
                   0 -0.15;
                   -0.3 0];
    
% drawObject(obj4);
% hold on

translated_object = translateObject(obj4, 1, 1,2.384);
figure(1)
drawObject(translated_object);
hold on

%% Generate lines and perform scan
lines = [translated_object([1:end],:), translated_object([2:end,1],:)];

scan = laserscanHighresol(x, y, theta, lines', maxDistance, resol,field_of_view);
r_noise_samples = normrnd(0,r_noise,1,size(scan,2));
theta_noise_samples = normrnd(0,theta_noise,1,size(scan,2));
scan_noisy = scan + [theta_noise_samples;r_noise_samples];
% figure
% polarplot(scan_noisy(1,:), scan_noisy(2,:))
%% Convert to polar coordinates and filter
cart = polar2carth(scan_noisy);
index = cart(1,:) <=xmax & cart(1,:) >= xmin & cart(2,:) <=ymax & cart(2,:) >=ymin;
cart_box = cart(:, index);

figure(1)
scatter(cart_box(1,:), cart_box(2,:));
axis([0.5 1.5 0.5 1.5]);

%% Run function as test
% [pose, type] = detectObject(cart_box)


%% Run ransac
ransac_params = [2, 100, 0.01, 3, 3];
box_lines = ransacLines(cart_box, ransac_params);
numlines = size(box_lines,2);
figure(3)
for idx=1:size(box_lines,2)
    plotLine(box_lines(:,idx), 'r', '-')
    hold on
end

drawObject(translated_object);
scatter(cart_box(1,:), cart_box(2,:));


if (numlines < 2)
    return;
end

%% Get intersection point of lines
A = [cos(box_lines(1,1)) sin(box_lines(1,1));
     cos(box_lines(1,2)) sin(box_lines(1,2))];
b = box_lines(2,:)';
intersect_point = A\b;

%% Get side lengths
c1 = cart_box(:,1);
c2 = cart_box(:,end);

lengths = [norm(c1 - intersect_point);
           norm(c2 - intersect_point)];
       
if (lengths(1) > lengths(2)) % Sort side lengths
%     lengths = lengths([2,1]);
%     ctemp = c2;
%     c2 = c1;
%     c1 = ctemp;
    cLong = c1;
else 
    cLong = c2;
end

dists=cos(box_lines(1,:))*cLong(1)+sin(box_lines(1,:))*cLong(2)-box_lines(2,:);
[~, min_index] = min(dists);

longLine = box_lines(:,min_index);

%% Match to object
detectIndex = matchObject(lengths, objLengths, box_lines(1,:), 1.5)
objectIndex = objIds(detectIndex)

%% Calculate orientation of object
figure(3)

piece_angle = longLine(1) + angleOffsets(detectIndex);

if piece_angle < -pi
    piece_angle = piece_angle + 2 * pi;
end
if piece_angle > pi
    piece_angle = piece_angle - 2*pi;
end

%% Calculate origin position of object
R = [cos(piece_angle) -sin(piece_angle);
     sin(piece_angle) cos(piece_angle)];
objectPosition = intersect_point + R*positionOffsets(detectIndex,:)'

%% Plot results
figure(3)
scatter(objectPosition(1), objectPosition(2))
arrowSize = 0.1;
arrowEnd = [cos(piece_angle);sin(piece_angle)]*arrowSize;
% annotation('arrow', [objectPosition(1); arrowEnd(1)], [objectPosition(2); arrowEnd(2)]);
quiver(objectPosition(1), objectPosition(2), arrowEnd(1), arrowEnd(2),0);
axis([0.5 1.5 0.5 1.5]);
% plotLine([piece_angle, 0], 'g', '--')