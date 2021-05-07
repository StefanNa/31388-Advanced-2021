close all
clear
clc

%% Real scan data
X_points=[1.802001, 1.801863, 1.802379, 1.801999, 1.802282, 1.802450, 1.802505, 1.802446, 1.802275, 1.801992, 1.802406, 1.801904, 1.817600, 1.835788, 1.854030, 1.872327, 1.891504, 1.910738, 1.930027, 1.950210, 1.970451, 1.991595, 2.012799, 2.034063, 2.056240, 2.079339, 2.102503, 2.125730, 2.149887, 2.174982, 2.200145, 1.802069, 1.801916, 1.801639, 1.802021, 1.802289, 1.801653, 1.801689, 1.802410, 1.802224, 1.801926, 1.802326, 1.802622, 1.819948, 1.838144, 1.856394, 1.874697, 1.893881, 1.913123, 1.932419, 1.952608, 1.973699, 1.994006, 2.015218, 2.037342, 2.059529, 2.081777, 2.104947, 2.129047, 2.153214, 2.178318, 1.800372, 1.821534, 1.841791, 1.861991, 1.882132, 1.901380, 1.919743, 1.938881, 1.957138, 1.975342, 1.992675, 2.009958, 2.027189, 2.043565, 2.060692, 2.076967, 2.092401, 2.108578, 2.123918, 2.139211, 2.153681, 2.168881, 2.183262, 2.197599, 2.197455, 2.197421, 2.197497, 2.196936, 2.197236, 2.196910, 2.197438, 2.197348, 2.197379, 2.196810, 2.197089, 1.805575, 1.826720, 1.846960, 1.867143, 1.886432, 1.905666, 1.924844, 1.943140, 1.961382, 1.979571, 1.996890, 2.014158, 2.031375, 2.047736, 2.064049, 2.080312, 2.096526, 2.111900, 2.127229, 2.142510, 2.157743, 2.172156, 2.186525, 2.196271, 2.196151, 2.196141, 2.196240, 2.196451, 2.196029, 2.196466, 2.196280, 2.196215, 2.196272, 2.196450, 2.196034, 2.196375, 2.196320, 2.196331, 2.196410, 2.196555, 2.195864, 2.196138, 2.196478, 2.195973, 2.196440, 2.196057, 2.196650, 2.196388, 2.196185, 2.196040, 2.195954, 2.195926, 2.193161, 2.171763, 2.150320, 2.127896, 2.104487, 2.080089, 2.054699, 2.028313, 2.001876, 1.973491, 1.945053, 1.914656, 1.882294, 1.849874, 1.815480, 2.196900, 2.196826, 2.196818, 2.196878, 2.197004, 2.196295, 2.196550, 2.196871, 2.196348, 2.196796, 2.196395, 2.196970, 2.196690, 2.196469, 2.196306, 2.196202, 2.196157, 2.196169, 2.174778, 2.153341, 2.130924, 2.107522, 2.083131, 2.058691, 2.032314, 2.005885, 1.978457, 1.949077, 1.919642, 1.888245, 1.854879, 1.821454, 2.171824, 2.134227, 2.097650, 2.063048, 2.029460, 1.996884, 1.966268, 1.935707, 1.907098, 1.879487, 1.851929, 1.826307, 1.805425, 1.806097, 1.805766, 1.806301, 1.805837, 1.806233, 1.806559, 1.805893, 1.806079, 1.806194, 1.806236, 1.806207, 1.807015, 1.806837, 1.806587, 1.807165, 2.169470, 2.131872, 2.095294, 2.060690, 2.027101, 1.994525, 1.963908, 1.933347, 1.904737, 1.877125, 1.849567, 1.823943, 1.803054, 1.803682, 1.803309, 1.803801, 1.803295, 1.803648, 1.803930, 1.803221, 1.803363, 1.803434, 1.803432, 1.803359, 1.804120, 1.803898, 1.803603, 1.804134, ];
Y_points=[1.575887, 1.561908, 1.548603, 1.534710, 1.521483, 1.508295, 1.495145, 1.482035, 1.468966, 1.455939, 1.443543, 1.430597, 1.429272, 1.429508, 1.429552, 1.429404, 1.429622, 1.429635, 1.429443, 1.429589, 1.429518, 1.429761, 1.429775, 1.429560, 1.429630, 1.429969, 1.430054, 1.429885, 1.429955, 1.430248, 1.430261, 1.574351, 1.560377, 1.546448, 1.533189, 1.519967, 1.506170, 1.493030, 1.480534, 1.467470, 1.454448, 1.442057, 1.429699, 1.429518, 1.429728, 1.429747, 1.429574, 1.429765, 1.429752, 1.429534, 1.429652, 1.430090, 1.429768, 1.429754, 1.430031, 1.430067, 1.429861, 1.429916, 1.430216, 1.430250, 1.430505, 1.429014, 1.428876, 1.429049, 1.429005, 1.428745, 1.428824, 1.429260, 1.428936, 1.428980, 1.428832, 1.429074, 1.429136, 1.429018, 1.429317, 1.428846, 1.428804, 1.429206, 1.428836, 1.428920, 1.428850, 1.429255, 1.428881, 1.428993, 1.428962, 1.441143, 1.453362, 1.465619, 1.478577, 1.490912, 1.503955, 1.516364, 1.529489, 1.542655, 1.556554, 1.569805, 1.427757, 1.427572, 1.427699, 1.427610, 1.427855, 1.427895, 1.427731, 1.427929, 1.427936, 1.427750, 1.427955, 1.427980, 1.427825, 1.428088, 1.428183, 1.428110, 1.427871, 1.428082, 1.428137, 1.428037, 1.427783, 1.428011, 1.428095, 1.431913, 1.444108, 1.456341, 1.468613, 1.480920, 1.493934, 1.506317, 1.519414, 1.532552, 1.545732, 1.558953, 1.572910, 1.431663, 1.440505, 1.449325, 1.458121, 1.466895, 1.475212, 1.483943, 1.492650, 1.500916, 1.509578, 1.517809, 1.526424, 1.534619, 1.542798, 1.550961, 1.559108, 1.567237, 1.574258, 1.574145, 1.574283, 1.574328, 1.574297, 1.574210, 1.574084, 1.573939, 1.574108, 1.573972, 1.574175, 1.574123, 1.573851, 1.573969, 1.573917, 1.429616, 1.438457, 1.447277, 1.456074, 1.464847, 1.473163, 1.481895, 1.490602, 1.498867, 1.507530, 1.515760, 1.524376, 1.532571, 1.540750, 1.548913, 1.557059, 1.565188, 1.573300, 1.573122, 1.573196, 1.573174, 1.573076, 1.572918, 1.573049, 1.572823, 1.572910, 1.573001, 1.572807, 1.572965, 1.572887, 1.572610, 1.572735, 1.586084, 1.585671, 1.585424, 1.585031, 1.584755, 1.584576, 1.584165, 1.584119, 1.583792, 1.583476, 1.583487, 1.583144, 1.581371, 1.571731, 1.562422, 1.552734, 1.543389, 1.533653, 1.523893, 1.514494, 1.504690, 1.494861, 1.485009, 1.475134, 1.464819, 1.454894, 1.444947, 1.434546, 1.577816, 1.577572, 1.577489, 1.577252, 1.577126, 1.577093, 1.576820, 1.576911, 1.576713, 1.576521, 1.576656, 1.576429, 1.574749, 1.565106, 1.555799, 1.546108, 1.536765, 1.527028, 1.517266, 1.507871, 1.498065, 1.488236, 1.478384, 1.468509, 1.458191, 1.448267, 1.438321, 1.427917, ];
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

x = 2;
y = 1.5;
theta = 0;
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
    theta_robot = robot_poses(poseIdx,3);
    A = [cos(theta_robot) -sin(theta_robot);
         sin(theta_robot) cos(theta_robot)];
    scan = laserscanHighresol(robot_poses(poseIdx,1), robot_poses(poseIdx,2), robot_poses(poseIdx,3), lines', maxDistance, resol,field_of_view);
    r_noise_samples = normrnd(0,r_noise,1,size(scan,2));
    theta_noise_samples = normrnd(0,theta_noise,1,size(scan,2));
    scan_noisy = scan + [theta_noise_samples;r_noise_samples];
    scan_cart = polar2carth(scan_noisy);
    scan_world = A*scan_cart + robot_poses(poseIdx,1:2)';
    scan_total(:,size(scan_total,2)+1:size(scan_total,2)+size(scan_world,2)) = scan_world;
end


scan_box_index = scan_total(1,:) <=x+xmax & scan_total(1,:) >=x+xmin & scan_total(2,:) <=y+ymax & scan_total(2,:) >=y+ymin;
scan_box = scan_total(:,scan_box_index);
% scatter(scan_total(1,:), scan_total(2,:))

%% Load real data
scan_box = [X_points; Y_points];
scatter(scan_box(1,:), scan_box(2,:));

%% Run ransac
ransac_params = [4, 30000, 0.01, 10, 10];
box_lines = ransacLines(scan_box, ransac_params);

numlines = size(box_lines,2);
figure(3)
for idx=1:size(box_lines,2)
    plotLine(box_lines(:,idx), 'r', '-')
    hold on
end

% drawObject(translated_object);
scatter(scan_box(1,:), scan_box(2,:), 'r');

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
%         r1 = box_lines(2,idx);
%         r2 = box_lines(2,jdx);
%         alpha1 = box_lines(1,idx);
%         alpha2 = box_lines(1,jdx);
%         intersect_test_y = (r1*cos(alpha2)-r2*cos(alpha1))/(sin(alpha1)*cos(alpha2)-sin(alpha2)*cos(alpha1));
%         intersect_test_x = (r1-intersect_test_y*sin(alpha1))/cos(alpha1);
    end
end
cornerCount = cornerCount - 1;
%% Match object type and find pose
if cornerCount == 4
    pose = mean(intersectPoints,2);
    c1 = intersectPoints(:,1);
    c2s = intersectPoints(:,2:end);
    dists = sqrt(sum((c2s-c1).^2));
    [sortedDists,distIndex] = sort(dists);
    c2 = c2s(:,distIndex(2));
    side = c2 - c1;
    pose(3) = atan2(side(2),side(1));
    if pose(3) > pi/2
        pose(3) = pose(3) - pi;
    end
    if pose(3) < -pi/2
        pose(3) = pose(3) + pi;
    end
    if sortedDists(1) > 0.175
        objectType = 2;
    else
        objectType = 1;
    end
elseif cornerCount == 3
    minCorner = 1000;
    minCornerIdx = 1;
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
else
    pose = [0;0;0];
    objectType = -1;
end

%% Plot matching result
figure(3)
scatter(pose(1), pose(2),50, 'b', 'filled');
arrowSize = 0.1;
arrowEnd = [cos(pose(3));sin(pose(3))]*arrowSize;
quiver(pose(1), pose(2), arrowEnd(1), arrowEnd(2), 0, 'b', 'LineWidth', 3);
title("Detected object " + objectType);
axis([1.4 2.6 1.2 1.8]);
grid on
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