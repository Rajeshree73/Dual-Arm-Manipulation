clear all;

% Call the function written at the end of the code
% Add the path to include all the folders and subfolders in the directory
addpath(genpath(strcat(pwd,'/home/rajeshree/catkin_ws/src/manipulation')));
robot = createRigidBodyTree;
axes = show(robot);
axes.CameraPositionMode = 'auto';

baseName = robot.BaseName;
body = getBody(robot,'r_clav');
dhparams = [ 0 0 0 0;-0.0125 0 0.2120 1.0000];

newSubtree =  subtree(robot,'r_clav');
a = removeBody(newSubtree,'r_palm');

world = robotics.Joint('world','fixed');
setFixedTransform(world,dhparams(1,:),'dh');

jnt = robotics.Joint('r_clav');
setFixedTransform(jnt,dhparams(2,:),'dh');


% dhparams = [1.0000 0 0 -0.0125; 0 1.0000 0 0; 0 0 1.0000 0.2120; 0 0 0 1.0000];
% 
% setFixedTransform(jnt1,dhparams(:,4),'dh');
% body.Joint = jnt1;
% 
% addBody(robot,body1,'pelvis')


% body = getBody(robot,'r_clav'); 
% dhparams = [-0.0125 0 0.2120 1.0000];
% jointObj = robotics.Joint('r_arm_shz');
% setFixedTransform(jointObj,dhparams,'dh');
% body.Joint = jointObj;
% 
% newSubtree =  removeBody(robot,'r_clav');
% a = removeBody(newSubtree,'r_palm');
 
% body = getBody(robot,'r_clav');
% newSubtree =  removeBody(robot,'r_clav');
% a = removeBody(newSubtree,'r_palm');
% 
% 
% dhparams = [1.0000 0 0 -0.0125; 0 1.0000 0 0; 0 0 1.0000 0.2120; 0 0 0 1.0000];
% 
% jnt = robotics.Joint('jnt','fixed');
% 
% setFixedTransform(jnt,dhparams(:,4),'dh');
% replaceJoint(robot,'utorso',jnt);
% 
% 
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