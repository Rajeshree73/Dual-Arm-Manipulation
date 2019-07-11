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



% Create a subtree from l_clav which will by default give "utorso" as Base
tree =  removeBody(robot,'l_clav');
% Remove all the joints from l_palm to reduce computation
a = removeBody(tree,'l_palm');

% New rigid body is created in order to shift the base of the new tree to
% the origin which is now at 'utorso'
body1 = robotics.RigidBody('body1');

world = robotics.Joint('world');
world.HomePosition;
tform = trvec2tform([-0.0125, 0, 0.2120]);
setFixedTransform(world,tform);
body1.Joint = world;
% To adjust it new rigid body tree is created
newSubtree = robotics.RigidBodyTree;

addBody(newSubtree,body1,'base');

addSubtree(newSubtree,'body1',tree);

% Create a point for end effector position
wayPoints = [0.6 0.6 0.2];
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
    configSoln(i,:) = ik('l_hand',tform,weights,initialguess);
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
 
    % Return to its home configuration 
    homeConfig = robot.homeConfiguration;

    
end

     
    