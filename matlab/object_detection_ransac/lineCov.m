function [sigmazp] = lineCov(zw,pose,poseCov)
    global lsrRelPose
    
    h_j = [0 0 -1;
        -cos(zw(1)) -sin(zw(1))...
        -lsrRelPose(1)*sin(zw(1)-pose(3)) + lsrRelPose(2)*cos(zw(1)-pose(3))];
    
    sigmazp = h_j*poseCov*h_j';
end

