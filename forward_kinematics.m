% Import the Atlas Model and Display 
clear all;
clc;

% Add the path to include all the folders and subfolders in the directory
addpath(genpath(strcat(pwd,'/home/rajeshree/catkin_ws/src/manipulation')));
robot = createRigidBodyTree;
axes = show(robot);
axes.CameraPositionMode = 'auto';

% Perform kinematics on torso region. assigning some values to utorso,ltorso and mtorso. 
% create one empty matrix of 1*109 and assign the values

configSoln = zeros(1,109);

for i = 1:3
   configSoln(i) = 0.5;
end

% Visualize robot configuration
title('Atlas wayPoint tracking animation');
% axis([-0.1 0.4 -0.35 0.35 0 0.35]);
for i = 1:size(configSoln)
    show(robot,configSoln(i,:));
    %pause(0.1);
end
hold off;
