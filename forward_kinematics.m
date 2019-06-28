% Import the Atlas Model and Display 
clear all;
clc;

% Add the path to include all the folders and subfolders in the directory
addpath(genpath(strcat(pwd,'/home/rajeshree/catkin_ws/src/manipulation')));
robot = createRigidBodyTree;
axes = show(robot);
axes.CameraPositionMode = 'auto';
pause(0.1);
% Perform kinematics on torso region. assigning some values to utorso,ltorso and mtorso. 
% create one empty matrix of 1*109 and assign the values

configSoln = zeros(1,109);

for i = 4:10
   configSoln(i) = 0.5;
end

% Visualize robot configuration
title('Atlas forward kinematics animation');

for i = 1:size(configSoln)
    show(robot,configSoln(i,:));
    pause(0.01);
end
hold off;
