function [ poseOut, poseCovOut ] = measurementUpdate( poseIn, poseCovIn, matchResult )
%[ poseOut, poseCovOut ] =MEASUREMENTUPDATE ( poseIn, poseCovIn,
%matchResult ) perform extended Kalman filter measurement update on the
%estimated robot pose poseIn with covariance poseCovIn using a set of
%matched predicted and extracted laser scanner lines given in matchResult.
%The arguments are defined as:
%       poseIn: The estimated robot pose given as [x,y,theta]
%       poseCovIn: The estimated covariance matrix of the robot pose
%       matchResult: A (5xnoOfWorldLines) matrix whose columns are 
%       individual pairs of line matches. It is structured as follows:
%       matchResult = [ worldLine(1,1) , worldLine(1,2) ...  ]
%                     [ worldLine(2,1) , worldLine(2,2)      ]
%                     [ innovation1(1) , innovation2(1)      ]
%                     [ innovation1(2) , innovation2(2)      ]
%                     [ matchIndex1    , matchIndex2    ...  ]
%           Note that the worldLines are in the world coordinates!
%       
%       poseOut: The updated robot pose estimate
%       poseCovOut: The updated estimate of the robot pose covariance 
%       matrix 

    % Constants
    % The laser scanner pose in the robot frame is read globally(lsrRelpose)
    % The varAlpha and varR are the assumed variances of the parameters of
    % the extracted lines, they are also read globally
    global lsrRelPose varAlpha varR
    h = [];
    v = [];
    
    matches = 0;
    for idx=1:size(matchResult,2)
        if matchResult(5,idx) > 0
            matches = matches + 1;
            h(matches*2-1:matches*2,1:3) = getJacobian(matchResult(1,idx), poseIn(3));
            v(matches*2-1:matches*2,1) = matchResult(3:4,idx);
        end
    end
    
    if matches == 0
        poseOut = poseIn;
        poseCovOut = poseCovIn;
        return
    end
    
    sigma_r = diag(repmat([varAlpha varR], 1, matches));
    
    sigma_in = h*poseCovIn*h' + sigma_r;
    
    K = poseCovIn*h'/sigma_in;
    
    poseOut = poseIn + K*v;
    poseCovOut = poseCovIn - K*sigma_in*K';

end

function h_j = getJacobian(alpha_w, theta)

    global lsrRelPose

    h_j = [0 0 -1;
        -cos(alpha_w) -sin(alpha_w)...
        -lsrRelPose(1)*sin(alpha_w-theta) + lsrRelPose(2)*cos(alpha_w-theta)];
end
