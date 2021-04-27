function [ matchResult ] = match( pose, poseCov, worldLines, laserLines )
% [matchResult] = MATCH(pose,poseCov,worldLines,laserLines)
%   This function matches the predicted lines to the extracted lines. The
%   criterion for a match is the mahalanobis distance between the (alpha,
%   r) parameters of the predicted and the extracted lines. The arguments
%   are:
%       pose: The estimated robot pose given as [x,y,theta]
%       poseCov: The estimated covariance matrix of the robot pose
%       worldLines: Known world lines in world coordinates, given as
%       [alpha;r] for each line. Number of rows = number of lines
%       laserLines: Lines extracted from the laser scan. Given as [alpha;r]
%       for each line. Number of rows = number of lines
%
%       matchResult: A (5xnoOfWorldLines) matrix whose columns are 
%       individual pairs of line matches. It is structured as follows:
%       matchResult = [ worldLine(1,1) , worldLine(1,2) ...  ]
%                     [ worldLine(2,1) , worldLine(2,2)      ]
%                     [ innovation1(1) , innovation2(1)      ]
%                     [ innovation1(2) , innovation2(2)      ]
%                     [ matchIndex1    , matchIndex2    ...  ]
%           Note that the worldLines are in the world coordinates!

    % The varAlpha and varR are the assumed variances of the parameters of
    % the extracted lines, they are read globally.
    global varAlpha varR
    g = 2;
    %matchResult = [];

    R = diag([varAlpha, varR]);

    worldLaser = zeros(size(worldLines));
    worldLaser_cov = zeros(2, 2, size(worldLines,2));
    %sigmaIN_inv = zeros(2,2,size(worldLines,2));
    
    for ldx=1:size(worldLines,2)
        [worldLaser(:,ldx), worldLaser_cov(:,:,ldx)] = projectToLaser(worldLines(:,ldx),pose, poseCov);
        %sigmaIN_inv(:,:,ldx) = inv(worldLaser_cov(:,:,ldx) + R);
    end

    sigmaIN = worldLaser_cov + R;

    matchResult = zeros(5,size(worldLines,2));

    for wdx=1:size(worldLines,2)
        minMaha = 1e5000;
        minIndex = 0;
        for ldx=1:size(laserLines,2)
            v = laserLines(:,ldx) - worldLaser(:,wdx);
            mahalanobis = v'*(sigmaIN(:,:,wdx)\v);
            if mahalanobis < g^2 && mahalanobis < minMaha
                minMaha = mahalanobis;
                minIndex = ldx;
            end
        end
        if minIndex > 0
            v_max = laserLines(:,minIndex) - worldLaser(:,wdx);
        else 
            v_max = [0; 0];
        end
            matchResult(:,wdx) = [worldLines(:,wdx)', v_max', minIndex]';
    end
end
