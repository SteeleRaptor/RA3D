clear; clc; close all;

%% USER ROTATIONS (degrees)
yaw = 10;
pitch = 30;
roll = 20;

yaw2 = 45;
pitch2 = 0;
roll2 = 90;
[yaw2, pitch2, roll2] = convertZYX_Zup_to_Xup(yaw, pitch, roll);
% yaw2 = yaw
% pitch2 = pitch
% roll2 = roll
%% Convert to radians
yaw = deg2rad(yaw);
pitch = deg2rad(pitch);
roll = deg2rad(roll);

yaw2 = deg2rad(yaw2);
pitch2 = deg2rad(pitch2);
roll2 = deg2rad(roll2);
%% Rotation matrices
Rz = @(a)[cos(a) -sin(a) 0;
          sin(a)  cos(a) 0;
          0       0      1];

Ry = @(a)[cos(a) 0 sin(a);
          0      1 0;
         -sin(a) 0 cos(a)];

Rx = @(a)[1 0 0;
          0 cos(a) -sin(a);
          0 sin(a) cos(a)];

%% ZYX rotation in Z-UP frame
R_world = Rz(yaw)*Ry(pitch)*Rx(roll);

%% Frame transform (X-up -> Z-up)
T = [0 0 1;
     0 1 0;
    -1 0 0];

%% Rotation defined in X-UP frame
R_xup = Rz(yaw2)*Ry(pitch2)*Rx(roll2);

%% Convert to world frame
R_xup_world = T' * R_xup * T;

%% Cube vertices
V = [-1 -1 -1;
      1 -1 -1;
      1  1 -1;
     -1  1 -1;
     -1 -1  1;
      1 -1  1;
      1  1  1;
     -1  1  1];

F = [1 2 3 4;
     5 6 7 8;
     1 2 6 5;
     2 3 7 6;
     3 4 8 7;
     4 1 5 8];

colors = [1 0 0;
          0 1 0;
          0 0 1;
          1 1 0;
          1 0 1;
          0 1 1];

%% Rotate cubes
V1 = (R_world * V')';
V2 = (R_xup_world * V')' + [4 0 0]; % shift second cube

%% Plot
figure
hold on
axis equal
grid on
view(3)

for i=1:6
    patch('Vertices',V1,'Faces',F(i,:), ...
        'FaceColor',colors(i,:),'FaceAlpha',1)
end

for i=1:6
    patch('Vertices',V2,'Faces',F(i,:), ...
        'FaceColor',colors(i,:),'FaceAlpha',1)
end

%% Draw world axes
quiver3(0,0,0,2,0,0,'r','LineWidth',2)
quiver3(0,0,0,0,2,0,'g','LineWidth',2)
quiver3(0,0,0,0,0,2,'b','LineWidth',2)

text(2,0,0,'X')
text(0,2,0,'Y')
text(0,0,2,'Z')

title('ZYX Rotation Comparison: Z-Up vs X-Up Frame')

function [yaw2, pitch2, roll2] = convertZYX_Zup_to_Xup(yaw, pitch, roll)

    yaw   = deg2rad(yaw);
    pitch = deg2rad(pitch);
    roll  = deg2rad(roll);

    % ZYX rotation
    R = rot_z(yaw) * rot_y(pitch) * rot_x(roll);

    T = [
        0 0 1;
        0 1 0;
       -1 0 0
    ];

    R_new = T * R * T';

    pitch2 = -asin(R_new(3,1));
    roll2  = atan2(R_new(3,2), R_new(3,3));
    yaw2   = atan2(R_new(2,1), R_new(1,1));

    yaw2   = rad2deg(yaw2);
    pitch2 = rad2deg(pitch2);
    roll2  = rad2deg(roll2);

end


function R = rot_x(a)
    c = cos(a); s = sin(a);
    R = [
        1 0 0;
        0 c -s;
        0 s  c
    ];
end

function R = rot_y(a)
    c = cos(a); s = sin(a);
    R = [
        c 0 s;
        0 1 0;
       -s 0 c
    ];
end

function R = rot_z(a)
    c = cos(a); s = sin(a);
    R = [
        c -s 0;
        s  c 0;
        0  0 1
    ];
end