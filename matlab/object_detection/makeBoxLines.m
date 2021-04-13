function lines=makeBoxLines(h,w,rot,disp)
Rz = [cos(rot) -sin(rot); sin(rot) cos(rot)];
xy0=Rz*[0,0]'+disp
xy1=Rz*[0,h]'+disp
xy2=Rz*[w,h]'+disp
xy3=Rz*[w,0]'+disp

% lines=[xy0',xy1';xy1',xy2';xy2',xy3';xy3',xy0'];
lines = [[xy0;xy1],[xy1;xy2],[xy2;xy3],[xy3;xy0]];
end
