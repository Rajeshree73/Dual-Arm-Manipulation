% Create rigid body tree from pelvis to Right arm for inverse kinematics 
% utorso is taken as a Basename and from there to the right end-effector rigid body is created to r_palm 
clear all;

% Call the function written at the end of the code
% Add the path to include all the folders and subfolders in the directory
%%
addpath(genpath(strcat(pwd,'/home/rajeshree/catkin_ws/src/manipulation')));
robot = createRigidBodyTree;
axes = show(robot);
axes.CameraPositionMode = 'auto';
show(robot);
%%
% Create new from r_clav rigid body tree and assign it as subtree. and then
% remove the end effectors and all fingers to remove the joints.

newSubtree =  removeBody(robot,'r_clav');
a = removeBody(newSubtree,'r_palm');
%%
% % Create a point for end effector position
% wayPoints = [0.5 -0.5 0.3];
% 
% plotWayPoints(wayPoints);
% trajectory = cscvn(wayPoints');
initialguess = newSubtree.homeConfiguration;
% 
% % Plot trajectory spline and waypoints
% hold on
% fnplt(trajectory,'r',2);
%%
%  Perform Generalized Inverse Kinematics for a point in space

gik = robotics.GeneralizedInverseKinematics('RigidBodyTree',newSubtree);

gik.ConstraintInputs = {'position','aiming'};

 

posTgt = robotics.PositionTarget('r_hand');
posTgt.TargetPosition = [0.5 -0.5 0.3];

aimCon = robotics.AimingConstraint('r_hand');
aimCon.TargetPoint = [0.0 0.0 0.0];

[q,solutionInfo] = gik(initialguess,posTgt,aimCon);
    
show(newSubtree,q);
title(['Solver status: ' solutionInfo.Status])
% axis([-0.75 0.75 -0.75 0.75 -0.5 1])

%%

% Create RigidBodyTree object for inverse kinematics
% Create tree-structured robot as it is representation of the robot structure


function [robot,homeConfig] = createRigidBodyTree

% Import the URDF file to load the robot

    robot = importrobot('/home/rajeshree/catkin_ws/src/ihmc_repos/matlab_ihmc_atlas_ros/urdf/atlas.urdf');
    
    
    % Add gravity 
    gravityVec = [0 0 -9.80665];
    robot.Gravity = gravityVec;
    % define Data Format for dynamic simulation
     robot.DataFormat = "row";
    % Add another massless coordinate frame for the end effector
%     eeOffset = 0.12;
%     eeBody = robotics.RigidBody('end_effector');
%     eeBody.Mass = 0;
%     eeBody.Inertia = [0 0 0 0 0 0];
%     setFixedTransform(eeBody.Joint,trvec2tform([eeOffset 0 0]));
%     addBody(robot,eeBody,'r_hand');
    % Return to its home configuration 
    homeConfig = robot.homeConfiguration;

    
end

     
    