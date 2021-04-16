%% clear
close all
clear
clc

%% Set up object data
robot_x = 0;
robot_y = 0;
theta = 0;
maxDistance = 2.5;
resol = 0.36;
field_of_view = 180;
theta_noise = 0.000;
r_noise = 0.000;

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
objects = zeros(4,4,2);
objects(1,:,:) = obj1;
objects(2,:,:) = obj2;
objects(3,1:3,:) = obj3;
objects(4,1:3,:) = obj4;

objectPositions = [1 1;
                   1 -1;
                   -1 1;
                   -1 -1]/2;
objectRotations = -pi:0.01:pi;
% objectPositions = [-1 1];
% objectRotations = 0;

% Initialize empty return values matrix (for speed)
poses = zeros(size(objects,1), size(objectPositions,1), length(objectRotations), 3);
detectTypes = zeros(size(objects,1), size(objectPositions,1), length(objectRotations));
referencePoses = zeros(size(objects,1), size(objectPositions,1), length(objectRotations), 3);
referenceTypes = zeros(size(objects,1), size(objectPositions,1), length(objectRotations));
%% Run evaluation loop
objectLength = size(objects,1);
positionLength = size(objectPositions,1);
rotationLength = length(objectRotations);
parfor objIdx=1:objectLength
    objectId = objIdx;
    obj = reshape(objects(objIdx,:,:), [],2);
    for positionIdx = 1:positionLength
        x = objectPositions(positionIdx,1);
        y = objectPositions(positionIdx,2);
        for rotationIdx = 1:rotationLength
            rotation = objectRotations(rotationIdx);
            %% Fill in reference data
            referencePoses(objIdx,positionIdx,rotationIdx,:) = [x y rotation];
            referenceTypes(objIdx,positionIdx,rotationIdx) = objectId;
            translated_object = translateObject(obj, x, y, rotation);
            %% Run laserscan
            if x < 0
                theta = pi;
            else
                theta = 0;
            end
            lines = [translated_object(1:end,:), translated_object([2:end,1],:)];
            scan = laserscanHighresol(robot_x, robot_y, theta, lines', maxDistance, resol,field_of_view);
            r_noise_samples = normrnd(0,r_noise,1,size(scan,2));
            theta_noise_samples = normrnd(0,theta_noise,1,size(scan,2));
            scan_noisy = scan + [theta_noise_samples;r_noise_samples];
            %% Convert to cartesian and filter
            A = [cos(theta) -sin(theta);
                 sin(theta) cos(theta)];
            cart = A*polar2carth(scan_noisy)+[robot_x;robot_y];
            
            index = cart(1,:) <=x+xmax & cart(1,:) >= x+xmin & cart(2,:) <=y+ymax & cart(2,:) >=y+ymin;
            cart_box = cart(:, index);

            %% Run object detection
            [poses(objIdx,positionIdx,rotationIdx,:), detectTypes(objIdx,positionIdx,rotationIdx)] = detectObject(cart_box);
        end
    end
end

%% Evaluate performance
numSims = objectLength*positionLength*rotationLength;
noDetect = detectTypes(:,:,:) == -1;
totalNoDetect = sum(noDetect, 'all');
noDetectPercent = totalNoDetect/numSims*100;
poseError = poses - referencePoses;
typeError = detectTypes - referenceTypes;
%% Create plots
for objIdx=1:objectLength
    figure(objIdx)
    detectTypesLocal = reshape(detectTypes(objIdx,:,:), positionLength, rotationLength);
    correctTypes = detectTypes(objIdx,:,:) == referenceTypes(objIdx,:,:) & ~noDetect(objIdx,:,:);
    wrongTypes = detectTypes(objIdx,:,:) ~= referenceTypes(objIdx,:,:) & ~noDetect(objIdx,:,:);
    correctTypes = reshape(correctTypes, positionLength, rotationLength);
    wrongTypes = reshape(wrongTypes, positionLength, rotationLength);
    posesx = reshape(poses(objIdx,:,:,1), positionLength, rotationLength);
    posesy = reshape(poses(objIdx,:,:,2), positionLength, rotationLength);
    scatter(posesx(correctTypes),posesy(correctTypes), 'b')
    hold on
    scatter(posesx(wrongTypes & detectTypesLocal == 1),posesy(wrongTypes & detectTypesLocal == 1), 'r')
    scatter(posesx(wrongTypes & detectTypesLocal == 2),posesy(wrongTypes & detectTypesLocal == 2), 'g')
    scatter(posesx(wrongTypes & detectTypesLocal == 3),posesy(wrongTypes & detectTypesLocal == 3), 'm')
    scatter(posesx(wrongTypes & detectTypesLocal == 4),posesy(wrongTypes & detectTypesLocal == 4), 'c')
    legend("Correct", "Obj1", "Obj2", "Obj3", "Obj4", "Location", "best")
    grid on
    xlabel('x [m]')
    ylabel('y [m]')
    
    %%Rotation plots
    figure(objIdx+objectLength)
    angles = reshape(poses(objIdx,:,:,3), positionLength, rotationLength);
    anglesReference = reshape(referencePoses(objIdx,:,:,3), positionLength, rotationLength);
    plot(anglesReference(1,correctTypes(2,:)), angles(1,correctTypes(2,:)), 'o');
end