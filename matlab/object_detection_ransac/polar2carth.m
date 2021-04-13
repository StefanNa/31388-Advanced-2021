function carth = polar2carth(pol)
% polar2carth(pol) Summary of this function goes here
%    Converts the polar coordinates
%    to carthesian coordinates x,y.

% r^2 = x^2 + y^2
% r*cos(theta) = x
% r*sin(theta) = y

% Extracting theta and r from input matrix
theta = pol(1,:);
r = pol(2,:);

carth(1,:) = r.*cos(theta);
carth(2,:) = r.*sin(theta);



end

