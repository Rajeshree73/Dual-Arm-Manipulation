clear all;

% Call the function written at the end of the code
% Add the path to include all the folders and subfolders in the directory
addpath(genpath(strcat(pwd,'/home/rajeshree/catkin_ws/src/manipulation')));
robot = createRigidBodyTree;
axes = show(robot);
axes.CameraPositionMode = 'auto';
%%

 
arm =  removeBody(robot,'r_clav');
a =  removeBody(arm,'r_palm');

newSubtree = robotics.RigidBodyTree;
body = robotics.RigidBody('body');
world = robotics.Joint('world');
body.Joint = world;
addBody(newSubtree,body,'base');
tform = trvec2tform([-0.0125, 0, 0.2120]);
setFixedTransform(world,tform);


body1 =  robotics.RigidBody('body1');
jnt = robotics.Joint('fixed');
body1.Joint = jnt;
tform1 = trvec2tform([-0.0125, 0, 0.2120]);
setFixedTransform(jnt,tform1);
addBody(newSubtree,body1,'body');

addSubtree(newSubtree,'body1',arm);

% jointObj = robotics.Joint('r_arm_shz');
% mdhparams =  [1.0000 0 0 0.1281; 0 1.0000 0 -0.2256; 0 0 1.0000 0.6896;0 0 0 1.0000];
% setFixedTransform(jointObj,mdhparams(4,:),'mdh');

% 
show(newSubtree);
% show(robot1);

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