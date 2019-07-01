% Create rigid body tree from pelvis to left arm for inverse kinematics 
% Pelvis is taken as a Basename and from there to the left end-effector rigid body is created 
clear all;

% Call the function written at the end of the code
% Add the path to include all the folders and subfolders in the directory
addpath(genpath(strcat(pwd,'/home/rajeshree/catkin_ws/src/manipulation')));
robot = createRigidBodyTree;
axes = show(robot);
axes.CameraPositionMode = 'auto';
base = robot.BaseName; 

% % Create new empty rigid body tree and assign it as subtree.
% left_arm = robotics.RigidBodyTree;
% 
% 
% body1 = getBody(robot,'ltorso');
% jnt1 = robotics.Joint('jnt1','revolute');
% replaceJoint(robot,'ltorso',jnt1);
% addBody(left_arm,body1,'base');
% 
% show(robot);
% showdetails(left_arm);
% newSubtree = subtree(robot,'l_clav');
% a = removeBody(newSubtree,'l_palm');
newSubtree = subtree(robot,'utorso');
a = removeBody(newSubtree,'l_palm');
b = removeBody(newSubtree,'r_clav');
c = removeBody(newSubtree,'l_situational_awareness_camera_link');
d = removeBody(newSubtree,'head');
c = removeBody(newSubtree,'r_situational_awareness_camera_link');
newJoint = robotics.Joint('fixed');
replaceJoint(newSubtree,'utorso',newJoint);



 
% Create a point for end effector position
% wayPoints = [0.5 0.5 -0.5;0.4 0.6 -0.4;0.3 0.7 -0.3;0.2 0.8 -0.2;0.1 0.9 -0.1;-0.1 1 0.1;-0.3 1.1 0.3;-0.5 1.2 0.5];
wayPoints = [0.6 0.6 0.2];
plotWayPoints(wayPoints);

trajectory = cscvn(wayPoints');

% Plot trajectory spline and waypoints
hold on
fnplt(trajectory,'r',2);

% Perform Inverse Kinematics for a point in space
ik = robotics.InverseKinematics('RigidBodyTree',newSubtree);
weights = [0.1 0.1 0 1 1 1];

initialguess = newSubtree.homeConfiguration;

numTotalPoints = 40;

% Evaluate trajectory to create a vector of end-effector positions
eePositions = ppval(trajectory,linspace(0,trajectory.breaks(end),numTotalPoints));
 
for i = 1:size(eePositions,2)
    tform = trvec2tform(eePositions(:,i)');
    configSoln(i,:) = ik('l_hand',tform,weights,initialguess);
    initialguess = configSoln(i,:);
end


% show(robot);

% Visualize the Atlas configurations
title('Atlas waypoint tracking visualization')

%axis([-0.1 0.4 -0.35 0.35 0 0.35]);
for i = 1:30
    show(newSubtree,configSoln(i,:));
    pause(0.1)
    clear ;
end
hold off





% Create RigidBodyTree object for inverse kinematics
% Create tree-structured robot as it is representation of the robot structure


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

     
    