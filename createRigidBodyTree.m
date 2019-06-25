% Create RigidBodyTree object for forward and reverse kinematics
% Create tree-structured robot as it is representation of the robot structure

function [robot,homeConfig] = createRigidBodyTree

% Import the URDF file to load the robot

    robot = importrobot('/home/rajeshree/catkin_ws/src/ihmc_repos/matlab_ihmc_atlas_ros/urdf/atlas.urdf');
    
    % Add gravity 
    gravityVec = [0 0 -9.80665];
    robot.Gravity = gravityVec;
 
    % Return to its home configuration 
    homeConfig = robot.homeConfiguration;

    
end


     
    