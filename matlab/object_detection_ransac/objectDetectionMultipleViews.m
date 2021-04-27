close all
clear
clc

%% Parameters
robot_poses = [0 1 0;
               1 2 -pi/2;
               1 0 pi/2;
               2 1 -pi];
maxDistance = 2.5;
resol = 0.36;
field_of_view = 180;
theta_noise = 0.000;
r_noise = 0.001;

x = 1;
y = 1.1;
theta = 0.4;
x_assume = 0.0;
y_assume = 0.0;
theta_assume=0;
poseRelBox = [-x_assume, -y_assume, -theta_assume];
A = [cos(-theta_assume) -sin(-theta_assume);
     sin(-theta_assume) cos(-theta_assume)];
poseRelBox(1:2) = A*poseRelBox(1:2)';
poseRelBox = poseRelBox';
xmin = -0.5;
xmax = 0.5;
ymin = -0.5;
ymax = 0.5;

poseCov = diag([0.5, 0.5, 0.5]);

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
              pi/2 .075;
              -pi 0.2;
              -pi/2 0.075];
          
obj3_param = [0 0;
              pi/2 0;
              pi/2-atan2(0.1,0.4) 0.4*sin(atan2(0.1,0.4))];
          
worldLines = obj1_param';
          
%% globals
global varAlpha;
global varR;
global lsrRelPose;
varAlpha = 0.001;
varR = 0.001;
lsrRelPose = [0, 0, 0];

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
ransac_params = [4, 20000, 0.01, 4, 2];
box_lines = ransacLines(scan_box, ransac_params);

numlines = size(box_lines,2);
figure(3)
for idx=1:size(box_lines,2)
    plotLine(box_lines(:,idx), 'r', '-')
    hold on
end

drawObject(translated_object);
scatter(scan_box(1,:), scan_box(2,:));

%% Find object corners
numBoxLines = size(box_lines,2);
intersectPoints = zeros(2,numBoxLines);
cornerCount = 1;
for idx=1:numBoxLines-1
    for jdx=idx+1:numBoxLines
        A = [cos(box_lines(1,idx)) sin(box_lines(1,idx));
             cos(box_lines(1,jdx)) sin(box_lines(1,jdx))];
        b = box_lines(2,[idx, jdx])';
        intersect = A\b;
        if(intersect(1) > 0 && intersect(1) < 10 && intersect(2) > 0 && intersect(2) < 10)
            intersectPoints(:,cornerCount) = intersect;
            cornerCount = cornerCount + 1;
        end
    end
end
cornerCount = cornerCount - 1;
%% Match object type and find pose
if cornerCount > 3
    pose = mean(intersectPoints,2);
    c1 = intersectPoints(:,1);
    c2s = intersectPoints(:,2:end);
    dists = sqrt(sum((c2s-c1).^2));
    [sortedDists,distIndex] = sort(dists);
    c2 = c2s(:,distIndex(2));
    side = c2 - c1;
    pose(3) = atan2(side(2),side(1));
    if sortedDists(1) > 0.175
        objectType = 2;
    else
        objectType = 1;
    end
else
    minCorner = 1000;
    minCornerIdx = 0;
    for idx=1:cornerCount
        c1 = intersectPoints(:,idx);
        c2s = intersectPoints(:,1:end ~= idx);
        sides = c2s - c1;
        dists = sqrt(sum(sides.^2));
        corner = dot(sides(:,1), sides(:,2));
        if corner < minCorner
            minCorner = corner;
            minCornerIdx = idx;
            [~,sideIndex] = max(dists);
            c2 = c2s(:,sideIndex);
        end
    end
    c1 = intersectPoints(:,minCornerIdx);
    side = c2 - c1;
    angle = atan2(side(2),side(1));
    pose = [c1; angle];
    if sqrt(sum(side.^2)) > 0.35
        objectType = 3;
    else
        objectType = 4;
    end
end

%% Plot matching result
figure(3)
scatter(pose(1), pose(2),50, 'b', 'filled');
arrowSize = 0.1;
arrowEnd = [cos(pose(3));sin(pose(3))]*arrowSize;
quiver(pose(1), pose(2), arrowEnd(1), arrowEnd(2), 0, 'b', 'LineWidth', 3);
title("Detected object " + objectType);
axis([0.5 1.5 0.5 1.5]);
%% Match line parameters
% numWorldLines = size(worldLines,2);
% numBoxLines = size(box_lines,2);
% % numBoxLines = 4;
% indexes = 1:numWorldLines;
% permutations = perms(indexes);
% minError = 1e10;
% minErrorPose = [0;0;0];
% lineRWeight = 1;
% lineAlphaWeight = 1;
% 
% alpha = box_lines(1,:);
% r = box_lines(2,:);
% 
% A = [cos(alpha)' sin(alpha)'];
% b = r';
% box_center = (A'*A)\A'*b;
% 
% [box_centered_alpha, box_centered_r] = lineTransform(box_lines(1,:), box_lines(2,:), -box_center(1), -box_center(2),0);
% box_centered = [box_centered_alpha; box_centered_r];
% % Old least squares fitting entire pose at once
% [~, box_centered_sorted_index] = sort(box_centered(2,:));
% box_centered_sorted = box_centered(:,box_centered_sorted_index);
% 
% box_angle = box_centered_sorted(1,3);

%% Transform object using calculated pose
% figure(3)
% % worldLaser = zeros(size(worldLines));
% % worldLaser_cov = zeros(2, 2, size(worldLines,2));
% [worldLaser_a, worldLaser_r] = lineTransform(worldLines(1,:), worldLines(2,:), box_center(1), box_center(2), box_angle);
% worldLaser = [worldLaser_a; worldLaser_r];
% for ldx=1:size(worldLines,2)
%         
% %         worldLaser(:ldx) = [worldLaser
% %         [worldLaser(:,ldx), worldLaser_cov(:,:,ldx)] = projectToLaser(worldLines(:,ldx),[box_center; box_angle], poseCov);
%         %sigmaIN_inv(:,:,ldx) = inv(worldLaser_cov(:,:,ldx) + R);
%         plotLine(worldLaser(:,ldx), 'm', '--');
% end
% axis([0.5 1.5 0.5 1.5]);

%% Plot parameter space
% figure(4)
% scatter(box_lines(1,:), box_lines(2,:));
% hold on
% scatter(worldLaser(1,:), worldLaser(2,:));
% xlabel('alpha [rad]')
% ylabel('r [m]');
% grid on
% legend("ransac", "obj map")

% figure(5);
% for idx=1:100
%     matches = match(poseRelBox', poseCov, worldLines, box_lines);
%     numMatches = 0;
%     for mIdx=1:size(matches,2)
%         if(matches(5,mIdx) > 0)
%             numMatches = numMatches+1;
%         end
%     end
%     numMatches
% 
%     [newPose, newPoseCov] = measurementUpdate(poseRelBox, poseCov, matches);
%     poseRelBox = newPose;
%     poseCov = newPoseCov*10;
%     worldLaser = zeros(size(worldLines));
%     worldLaser_cov = zeros(2, 2, size(worldLines,2));
%     
%     if(mod(idx,10) == 0)
%         clf
%         hold on
%         for ldx=1:size(worldLines,2)
%                 [worldLaser(:,ldx), worldLaser_cov(:,:,ldx)] = projectToLaser(worldLines(:,ldx),poseRelBox, poseCov);
%                 %sigmaIN_inv(:,:,ldx) = inv(worldLaser_cov(:,:,ldx) + R);
%                 plotLine(worldLaser(:,ldx), 'm', '--');
%         end
%         for laserIdx=1:size(box_lines,2)
%             plotLine(box_lines(:,laserIdx), 'r', '-');
%         end
%     end
% end
% 
% figure(6)
% scatter(box_lines(1,:), box_lines(2,:));
% hold on
% scatter(worldLaser(1,:), worldLaser(2,:));
% xlabel('alpha [rad]')
% ylabel('r [m]');
% grid on
% legend("ransac", "obj map")