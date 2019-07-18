clear all;
% Add robot to the virtual world

% Import URDF
robot = importrobot('/home/rajeshree/catkin_ws/src/ihmc_repos/matlab_ihmc_atlas_ros/urdf/atlas.urdf');

% Create vr world , it will be parent
robotWorld = vrworld('robot_scene.wrl','new');
open(robotWorld);

% Add robot to the existing vr world and robot will be root
n = vrinsertrobot(robotWorld,robot);
% update the scene and then open the world 
vrdrawnow
view(robotWorld);

% Get transforms of the current pose of the robot
[node, W, tforms] = vrinsertrobot(robotWorld, robot);
vrfigure(robotWorld);

vrupdaterobot(robot, tforms, homeConfiguration(robot));
vrdrawnow;
vrfigure(robotWorld);