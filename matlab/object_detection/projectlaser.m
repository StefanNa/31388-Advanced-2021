close all
clc
clear all

% set box size and rotation
h=0.2;
w=0.2;
rot=pi/10;
disp=[0.5;0.5];
Rz = [cos(rot) -sin(rot); sin(rot) cos(rot)];

%function that creates the box lines to be used by the laser scanner function
lines=makeBoxLines(h,w,rot,disp);
xy=[lines(1,1:2);lines(1,3:4);lines(3,1:2);lines(3,3:4)];


maxDistance=2.5;
resol=0.36;
field_of_view=180;
[x, y, theta]=deal(-0.2,0,0);
rotxy=Rz*[x;y];
x_box=xy(:,1)-rotxy(1);
y_box=xy(:,2)-rotxy(2);

% add noise
scan = laserscanHighresol(x, y, theta, lines, maxDistance, resol,field_of_view);
noise=normrnd(0,0.01,size(scan));
scan=scan;

figure(1)
polarplot(scan(1,:),scan(2,:))

cart = polar2carth(scan);
figure(2)
hold on;
scatter(cart(1,:),cart(2,:))
scatter(x_box,y_box)
scatter(x,y)
grid on;
axis equal;

%% remove the measurements outside of area of interest
index = cart(1,:) <=1 & cart(1,:) >= 0 & cart(2,:) <=1 & cart(2,:) >=0;
cart_box = cart(:,index);
figure(3)
scatter(cart_box(1,:), cart_box(2,:))
hold on

%% Run ransac on resulting data to try and find two lines
ransac_params = [2, 20, 0.05, 3, 3];
box_lines = ransacLines(cart_box, ransac_params);
figure(3)
for idx=1:size(box_lines,2)
    plotLine(box_lines(:,idx), 'r', '-')
end
scatter(x_box, y_box)

%% fit line in points per edge with polyfit
p1=polyfit(c1(1,:),c1(2,:),1)
x__1 = linspace(0,1.5);
y__1 = polyval(p1,x__1);

p2=polyfit(c2(1,:),c2(2,:),1)
x__2 = linspace(0,1.5);
y__2 = polyval(p2,x__2);

figure(3)
plot(c1(1,:),c1(2,:),'o')
hold on;
plot(x__1,y__1)
plot(c2(1,:),c2(2,:),'*')
plot(x__2,y__2)
hold off;
axis equal;
%% get rotation and position
line1 = lsqlinefunction(c1)
rotation1=line1(1)
position1=line1(2)
fprintf("Position of box = " + string(position1) + " and rotation = " + string(rad2deg(rotation1)))
line2 = lsqlinefunction(c2)
rotation2=line2(1)
position2=line2(2)
fprintf("Position of box = " + string(position2) + " and rotation = " + string(rad2deg(rotation2)))

figure(4)
plot(c1(1,:),c1(2,:),'o')
hold on;
% plot(x__1,y__1)
plot(c1(1,:),c1(2,:),'x')
% plot(x__2,y__2)
plot(c2(1,:),c2(2,:),'*')
% quiver(0,0,cos(rotation1),sin(rotation1),0.1)
% quiver(0,0,cos(rotation2+pi/2),sin(rotation2+pi/2),0.1) 
grid on;
axis equal;
hold off;
%% 5.1
% scan_max = scan(:,scan(2,:)<maxDistance)
% figure(3)
% polarplot(scan_max(1,:),scan_max(2,:))
% 
% c = polar2carth(scan_max);
% [position, rotation] = lsqline(c)
% fprintf("Position of box = " + string(position) + " and rotation = " + string(rad2deg(rotation)))
% 
% figure(4)
% hold on;
% plot(c(1,:),c(2,:),'x')
% quiver(0,0,cos(rotation),sin(rotation),0.1) 
% grid on;
% axis equal;
% hold off;

%%

% move points to origin
% rotate to align with base
% find center
% move and rotate center

transl=c1(:,idx)
rot=-1*(rotation1+rotation2)/2
Rz = [cos(rot) -sin(rot); sin(rot) cos(rot)];
c1_=Rz*(c1-transl)
c2_=Rz*(c2-transl)

center_=(max(c2_,[],2)+max(c1_,[],2))/2

rot=(rotation1+rotation2)/2
Rz = [cos(rot) -sin(rot); sin(rot) cos(rot)];

center=Rz*center_+transl

figure(2)
hold on;
scatter(x_box,y_box)
scatter(x,y)
scatter(center(1),center(2))
grid on;
axis equal;

