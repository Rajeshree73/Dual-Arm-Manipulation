% Import the Atlas Model and Display 
clear all;
clc;

% Add the path to include all the folders and subfolders in the directory
addpath(genpath(strcat(pwd,'/home/rajeshree/catkin_ws/src/manipulation')));
robot = createRigidBodyTree;
axes = show(robot);
axes.CameraPositionMode = 'auto';

% Create the set of desired wayPoints

%wayPoints = [1 -1 0.1;0.75 0 1.4;0.75 0.25 1; 0.75 0.45 0.75;0.5 0.6 0.5;0.5 0.5 1;1.25 0 0.75; 1 1 0.1];
wayPoints = [.5 .5 .5;0.75 0.25 -0.25];
plotWayPoints(wayPoints);

% Create a smooth curve from the wayPoints to track the trajectory using a curve fitting toolbox function cscvn

trajectory = cscvn(wayPoints');

% Plot the trajectory spline joining the waypoints
hold on;
fnplt(trajectory,'m',1);

% Perfrom Inverse Kinematics for a point in space using "ik" solver
ik = robotics.InverseKinematics('RigidBodyTree',robot);

% Define all the parameters associated with this algorithm called structure

% weights for pose tolerance , orientation and translation respectively

% we might need to change the weight on z if we are not getting solution
weights = [0.1 0.1 1 1 1 1];
initialguess = [.5 .5 .5];
numTotalPoints = 30;

% Evaluate trajectory to create a vector of end-effector positions
eePositions = ppval(trajectory,linspace(0,trajectory.breaks(end),numTotalPoints));

% Call Inverse Kinematics solver for every end-effector position using the
% previous configuration as initial guess
for i= 1:size(eePositions,2)
    tform = trvec2tform(eePositions(:,i)');
    configSoln(i,:) = ik('l_end_eff',tform,weights,initialguess);
    initialguess = configSoln(i,:);
end

% Visualize robot configuration
title('Atlas wayPoint tracking animation');
axis([-0.1 0.4 -0.35 0.35 0 0.35]);
for i = 1:size(eePositions,2);
    show(robot,configSoln(i,:),'PreservePlot',false,'Frames','off');
    pause(0.1);
end
hold off;




