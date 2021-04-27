function [ projectedLine, outCov ] = projectToLaser( worldLine,poseIn, covIn)
%[projectedLine, lineCov] = PROJECTTOLASER(worldLine,poseIn,covIn) 
%Project a word line to the laser scanner frame given the
%world line, the robot pose and robot pose covariance. Note that the laser
%scanner pose in the robot frame is read globally
%   worldLine: The line in world coordinates
%   poseIn: The robot pose
%   covIn: The robot pose covariance
%
%   projectedLine: The line parameters in the laser scanner frame
%   lineCov: The covariance of the line parameters

%% Constants
global lsrRelPose % The laser scanner pose in the robot frame is read globally


%% Calculation
alpha_w = worldLine(1);
r_w = worldLine(2);
x = poseIn(1);
y = poseIn(2);
theta = poseIn(3);

alpha_r = alpha_w - theta;
r_r = r_w-x*cos(alpha_w) - y*sin(alpha_w);

alpha_l = alpha_r - lsrRelPose(3);
x_l = lsrRelPose(1);
y_l = lsrRelPose(2);
projectedLine = [alpha_l, r_r - x_l*cos(alpha_r) - y_l*sin(alpha_r)];
if projectedLine(2) < 0
    projectedLine(2) = -projectedLine(2);
    projectedLine(1) = projectedLine(1) + pi;
end
if(projectedLine(1) > pi)
    projectedLine(1) = projectedLine(1) - 2*pi;
end
if(projectedLine(1) < -pi)
    projectedLine(1) = projectedLine(1) + 2*pi;
end

outCov = lineCov(projectedLine, poseIn, covIn);
end
