% Create rigid body tree from pelvis to left arm for inverse kinematics 
% utorso is taken as a Basename and from there to the left end-effector rigid body is created 
clear all;

% Call the function written at the end of the code
% Add the path to include all the folders and subfolders in the directory
addpath(genpath(strcat(pwd,'/home/rajeshree/catkin_ws/src/manipulation')));
robot = createRigidBodyTree;
axes = show(robot);
axes.CameraPositionMode = 'auto';
base = robot.BaseName; 

% Call the function to create a subtree
initial_BodyName = 'r_clav';
final_BodyName = 'r_palm';
[newSubtree] = subtree_generation(initial_BodyName,final_BodyName);

% Create the set of desired wayPoints
wayPoints = [-0.7 -1.0 0.55;-0.5 -0.9 0.6;-0.3 -0.8 0.75;-0.1 -0.7 0.7;0.1 -0.6 0.75;0.3 -0.5 0.6;0.5 -0.45 0.65;0.7 -0.4 0.5];
plotWayPoints(wayPoints);

% Create a smooth curve from the wayPoints to track the trajectory using a curve fitting toolbox function cscvn
trajectory = cscvn(wayPoints');
% hold off;
% Plot the trajectory spline joining the waypoints
 hold on
fnplt(trajectory,'r',2);
% 
% Perfrom Inverse Kinematics for a point in space using "ik" solver
% Define all the parameters associated with this algorithm called structure
% weights for pose tolerance , orientation and translation respectively
% we might need to change the weight on z if we are not getting solution
ik = robotics.InverseKinematics('RigidBodyTree',newSubtree);
weights = [0.1 0.1 1 1 1 1];
initialguess = newSubtree.homeConfiguration;

numTotalPoints = 30;

% Evaluate trajectory to create a vector of end-effector positions
eePositions = ppval(trajectory,linspace(0,trajectory.breaks(end),numTotalPoints));

% Call Inverse Kinematics solver for every end-effector position using the
% previous configuration as initial guess

 for i= 1:size(eePositions,2)
     tform = trvec2tform(eePositions(:,i)');
     configSoln(i,:) = ik('r_hand',tform,weights,initialguess);
     initialguess = configSoln(i,:);
end 

% Visualize robot configuration
title('Atlas trajectory tracking animation');
%axis([-0.1 0.4 -0.35 0.35 0 0.35]);
for i = 1:size(eePositions,2);
   show(newSubtree,configSoln(i,:),'PreservePlot', false,'Frames','off');
   pause(0.1);
  
end
hold off;


function [robot,homeConfig] = createRigidBodyTree

% Import the URDF file to load the robot

    robot = importrobot('/home/rajeshree/catkin_ws/src/ihmc_repos/matlab_ihmc_atlas_ros/urdf/atlas.urdf');
    
    
    % Add gravity 
    gravityVec = [0 0 -9.80665];
    robot.Gravity = gravityVec;
    % define Data Format for dynamic simulation
    % robot.DataFormat = "row";
 
    % Return to its home configuration 
    homeConfig = robot.homeConfiguration;

    
end    
