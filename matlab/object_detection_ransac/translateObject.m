function newObject = translateObject(object,x, y, theta)
    
A = [cos(theta) -sin(theta);
    sin(theta) cos(theta)];
    newObject = (A*object')' + [x y];
end

