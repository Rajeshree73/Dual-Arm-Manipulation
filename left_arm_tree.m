% Create rigid body tree from pelvis to left arm for inverse kinematics 
% Pelvis is taken as a Basename and from there to the left end-effector rigid body is created 
clear all;

% left_arm_tree

robot = createRigidBodyTree;
base = robot.BaseName;
left_arm = robotics.RigidBodyTree;


body1 = getBody(robot,'ltorso');
jnt1 = robotics.Joint('jnt1','revolute');
replaceJoint(robot,'ltorso',jnt1);
addBody(left_arm,body1,'base');

show(robot);
showdetails(left_arm);



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

     
    