clear all;

% Call the function written at the end of the code
% Add the path to include all the folders and subfolders in the directory
addpath(genpath(strcat(pwd,'/home/rajeshree/catkin_ws/src/manipulation')));
robot = createRigidBodyTree;
axes = show(robot);
axes.CameraPositionMode = 'auto';
%%
tree =  removeBody(robot,'r_clav');
a = removeBody(tree,'r_palm');

body1 = robotics.RigidBody('body1');

jnt1 = robotics.Joint('jnt1','revolute');
jnt1.HomePosition = 0;
tform = trvec2tform([-0.0125, 0, 0.2120]);
setFixedTransform(jnt1,tform);
body1.Joint = jnt1;

newSubtree = robotics.RigidBodyTree;
addBody(newSubtree,body1,'base');

addSubtree(newSubtree,'body1',tree);
show(newSubtree);

%%

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