function [pose, objectType] = detectObject(scan_cart)
%%Parameters
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
%% Run ransac
ransac_params = [2, 100, 0.01, 3, 3];
box_lines = ransacLines(scan_cart, ransac_params);
numlines = size(box_lines,2);
% figure(3)
% for idx=1:size(box_lines,2)
%     plotLine(box_lines(:,idx), 'r', '-')
%     hold on
% end

%drawObject(translated_object);
% scatter(scan_cart(1,:), scan_cart(2,:));


if (numlines < 2)
    pose = [0 0 0];
    objectType = -1;
    return;
end

%% Get intersection point of lines
A = [cos(box_lines(1,1)) sin(box_lines(1,1));
     cos(box_lines(1,2)) sin(box_lines(1,2))];
b = box_lines(2,:)';
intersect_point = A\b;

%% Get side lengths
c1 = scan_cart(:,1);
c2 = scan_cart(:,end);

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
detectIndex = matchObject(lengths, objLengths, box_lines(1,:), 1.5);
objectIndex = objIds(detectIndex);

%% Calculate orientation of object
% figure(3)

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
objectPosition = intersect_point + R*positionOffsets(detectIndex,:)';
pose = [objectPosition' piece_angle];
objectType = objectIndex;
end

