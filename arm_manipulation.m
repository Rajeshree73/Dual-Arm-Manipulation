% Load the URDF file and diaplay the model
clc;
clear;
addpath(genpath(strcat(pwd,'/home/rajeshree/catkin_ws/src/ihmc_repos/ihmc_atlas_ros/urdf')))
robot = importrobot('atlas_v5_robotiq.urdf');
axes = show (robot);
axes.CameraPositionMode = 'auto';

points = [0.2 -0.2 0.02;0.25 0 0.15; 0.2 0.2 0.02];
size = size(points);

%% creating the set of the desired points
PlotPoints(points);

%% creating the smooth curve for trajectory 

traj = cscvn(points);

%% plot the trajectory spline and points
hold on;
fnplt (traj, 'r', 2);




%% creating the function for plotting the points

function PlotPoints(points)
 for i = 1:size(points, 1)
     PlotSpheres(0.01,points(i,:))
 end
end
 

%% creating the function to plot a sphere 

function PlotSpheres(size, pointPosition)

[x, y, z] = sphere(20);

% Translate to the specified postion

X = size*x + pointPosition(1);
Y = size*y + pointPosition(2);
Z = size*z + pointPosition(3);

% Add the sphere to the figure and configure lighting 

s = patch(surf2patch(X,Y,Z));
s.FaceColor = 'blue';
s.FaceLighting = 'gouraud';
s.EdgeAlpha = 0;

% move the light
lightObj = findobj(gca, 'Type', 'Light');
lightObj.Position = [1,1,1];
end








 
 
