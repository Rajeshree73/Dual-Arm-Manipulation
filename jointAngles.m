clear all;
clc;

% Add the path to include all the folders and subfolders in the directory
addpath(genpath(strcat(pwd,'/home/rajeshree/catkin_ws/src/manipulation')));
robot = createRigidBodyTree;

% Import the Atlas Model and Display 
axes = show(robot);
axes.CameraPositionMode = 'auto';

% Return to its home configuration 
homeConfig = robot.homeConfiguration;
    
