% General function to create a subtree from the robot 
% Assign initial and final body name 

function [newSubtree] = Subtree_generation(initial_BodyName,final_BodyName)

addpath(genpath(strcat(pwd,'/home/rajeshree/catkin_ws/src/manipulation')));
robot = createRigidBodyTree;
% axes = show(robot);
homeConfig = robot.homeConfiguration;
axes.CameraPositionMode = 'auto';
base = robot.BaseName; 

% Create a subtree from "initial_BodyName" which will by default give "utorso" as Base
tree =  removeBody(robot,initial_BodyName);
Base_initial_BodyName = tree.BaseName;
% To remove all the joints from of the gripper to reduce computation and
% type the end of the subtree "final_BodyName"
a = removeBody(tree,final_BodyName);

% New rigid body is created in order to shift the base of the new tree to
% the origin which is now at 'utorso'

body1 = robotics.RigidBody('body1');
world = robotics.Joint('world');
world.HomePosition;

% In  order to shift the base and attach to the New rigid body transform
% with respect to the "pelvis" is needed. 
tform = getTransform(robot,robot.homeConfiguration,Base_initial_BodyName);
setFixedTransform(world,tform);

body1.Joint = world;
% add required subtree to this new rigid body tree
newSubtree = robotics.RigidBodyTree;

addBody(newSubtree,body1,'base');

addSubtree(newSubtree,'body1',tree);
   


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
end