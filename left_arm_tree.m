% Create rigid body tree from pelvis to left arm for inverse kinematics 
% Pelvis is taken as a Basename and from there to the left end-effector rigid body is created 


function [robot,homeConfig] = left_arm_tree

robot = importrobot('/home/rajeshree/catkin_ws/src/ihmc_repos/matlab_ihmc_atlas_ros/urdf/atlas.urdf');
robot = robotics.RigidBodyTree;

% body0 = robotics.RigidBody('pelvis');
% jnt0 = robotics.Joint('jnt0','fixed');
% body0.Joint = jnt0;
% BaseName = 'pelvis';
% basename = robot.BaseName;
% addBody(robot,body0,basename)
% 
% body1 = robotics.RigidBody('b1');
% jnt1 = robotics.Joint('jnt1','revolute');
% body1.Joint = jnt1;
% addBody(robot,body1,'pelvis')

a = sub(robot,'ltorso');
addSubtree(robot,robot.BaseName,a);
% getBody(robot,'pelvis');
% addBody(robot,)


% Return to its home configuration 
homeConfig = robot.homeConfiguration;
showdetails(robot)

end