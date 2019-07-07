% Create rigid body tree from pelvis to Right arm for inverse kinematics 
% utorso is taken as a Basename and from there to the right end-effector rigid body is created to r_palm 
clear all;

% Call the function written at the end of the code
% Add the path to include all the folders and subfolders in the directory
addpath(genpath(strcat(pwd,'/home/rajeshree/catkin_ws/src/manipulation')));
robot = createRigidBodyTree;
axes = show(robot);
axes.CameraPositionMode = 'auto';


% Create new from r_clav rigid body tree and assign it as subtree. and then
% remove the end effectors and all fingers to remove the joints.
%  
% newSubtree = subtree(robot,'r_clav');
% base = newSubtree.BaseName;
% a = removeBody(newSubtree,'r_palm');
% %addSubtree(robot,'mtorso',newSubtree);


newSubtree = subtree(robot,'pelvis');
a = removeBody(newSubtree,'l_clav');
b = removeBody(newSubtree,'r_palm');
c = removeBody(newSubtree,'l_situational_awareness_camera_link');
d = removeBody(newSubtree,'head');
e = removeBody(newSubtree,'r_situational_awareness_camera_link');
f = removeBody(newSubtree,'l_uglut');
g = removeBody(newSubtree,'r_uglut');
h = removeBody(newSubtree,'imu_link');



% newJoint = robotics.Joint('back_bkx');
% replaceJoint(newSubtree,'utorso',newJoint);

newJoint = robotics.Joint('back_bkz');
replaceJoint(newSubtree,'ltorso',newJoint);

newJoint = robotics.Joint('back_bky');
replaceJoint(newSubtree,'mtorso',newJoint);


% Create a point for end effector position
wayPoints = [0.6 -0.6 0.2];
plotWayPoints(wayPoints);
trajectory = cscvn(wayPoints');

% Plot trajectory spline and waypoints
hold on
fnplt(trajectory,'r',2);

% Perform Inverse Kinematics for a point in space
ik = robotics.InverseKinematics('RigidBodyTree',newSubtree);
weights = [0.1 0.1 1 1 1 1];

initialguess = newSubtree.homeConfiguration;

numTotalPoints = 30;

% Evaluate trajectory to create a vector of end-effector positions
eePositions = ppval(trajectory,linspace(0,trajectory.breaks(end),numTotalPoints));

for i = 1:size(eePositions,2)
    tform = trvec2tform(eePositions(:,i)');
    configSoln(i,:) = ik('r_hand',tform,weights,initialguess);
    initialguess = configSoln(i,:);
end

% 
% show(robot);

% Visualize the Atlas configurations
title('Atlas waypoint tracking visualization')

%axis([-0.1 0.4 -0.35 0.35 0 0.35]);
for i = 1:30
    show(newSubtree,configSoln(i,:));
    pause(0.1)
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

     
    