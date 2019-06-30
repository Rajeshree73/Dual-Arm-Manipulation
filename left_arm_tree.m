% Create rigid body tree from pelvis to left arm for inverse kinematics 
% Pelvis is taken as a Basename and from there to the left end-effector rigid body is created 
clear all;

% Call the function written at the end of the code
% Add the path to include all the folders and subfolders in the directory
addpath(genpath(strcat(pwd,'/home/rajeshree/catkin_ws/src/manipulation')));
robot = createRigidBodyTree;
axes = show(robot);
axes.CameraPositionMode = 'auto';
base = robot.BaseName; 

% % Create new empty rigid body tree and assign it as subtree.
% left_arm = robotics.RigidBodyTree;
% 
% 
% body1 = getBody(robot,'ltorso');
% jnt1 = robotics.Joint('jnt1','revolute');
% replaceJoint(robot,'ltorso',jnt1);
% addBody(left_arm,body1,'base');
% 
% show(robot);
% showdetails(left_arm);
newSubtree = subtree(robot,'l_scap');

% Create a point for end effector position
wayPoints = [0.6 0.6 0.2];
plotWayPoints(wayPoints);

trajectory = cscvn(wayPoints');

% Plot trajectory spline and waypoints
hold on
fnplt(trajectory,'r',2);

% Perform Inverse Kinematics for a point in space
ik = robotics.InverseKinematics('RigidBodyTree',newSubtree);
weights = [0.1 0.1 0 1 1 1];
%initialguess = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0];
%initialguess =  struct('l_clav',0,'l_scap',0,'l_uarm',0,'l_larm',0,'l_ufarm',0,'l_lfarm',0,'l_hand',0,'l_palm',0,'l_finger_1_link_0',0,'l_finger_1_link_1',0,'l_finger_1_link_2',0,'l_finger_1_link_3',0,'l_finger_1_link_paradistal_hinge',0,'l_finger_1_link_median_actuating_hinge',0,'l_finger_1_link_median_bar',0,'l_finger_1_link_paramedian_hinge',0,'l_finger_1_link_median_bar_underactuated',0,'l_finger_1_link_paraproximal_actuating_hinge',0,'l_finger_1_link_paraproximal_bar',0,'l_finger_1_link_proximal_actuating_hinge',0,'l_finger_1_link_proximal_actuating_bar',0,'l_finger_2_link_0',0,'l_finger_2_link_1',0,'l_finger_2_link_2',0,'l_finger_2_link_3',0,'l_finger_2_link_paradistal_hinge',0,'l_finger_2_link_median_actuating_hinge',0,'l_finger_2_link_median_bar',0,'l_finger_2_link_paramedian_hinge',0,'l_finger_2_link_median_bar_underactuated',0,'l_finger_2_link_paraproximal_actuating_hinge',0,'l_finger_2_link_paraproximal_bar',0,'l_finger_2_link_proximal_actuating_hinge',0,'l_finger_2_link_proximal_actuating_bar',0,'l_finger_middle_link_0',0,'l_finger_middle_link_1',0,'l_finger_middle_link_2',0,'l_finger_middle_link_3',0,'l_finger_middle_link_paradistal_hinge',0,'l_finger_middle_link_median_actuating_hinge',0,'l_finger_middle_link_median_bar',0,'l_finger_middle_link_paramedian_hinge',0,'l_finger_middle_link_median_bar_underactuated',0,'l_finger_middle_link_paraproximal_actuating_hinge',0,'l_finger_middle_link_paraproximal_bar',0,'l_finger_middle_link_proximal_actuating_hinge',0,'l_finger_middle_link_proximal_actuating_bar',0,'l_end_eff',0);   

% field1 = ['l_clav','l_scap','l_uarm','l_larm','l_ufarm','l_lfarm','l_hand','l_palm','l_finger_1_link_0','l_finger_1_link_1','l_finger_1_link_2','l_finger_1_link_3','l_finger_1_link_paradistal_hinge','l_finger_1_link_median_actuating_hinge','l_finger_1_link_median_bar','l_finger_1_link_paramedian_hinge','l_finger_1_link_median_bar_underactuated','l_finger_1_link_paraproximal_actuating_hinge','l_finger_1_link_paraproximal_bar','l_finger_1_link_proximal_actuating_hinge','l_finger_1_link_proximal_actuating_bar','l_finger_2_link_0','l_finger_2_link_1','l_finger_2_link_2','l_finger_2_link_3','l_finger_2_link_paradistal_hinge','l_finger_2_link_median_actuating_hinge','l_finger_2_link_median_bar','l_finger_2_link_paramedian_hinge','l_finger_2_link_median_bar_underactuated','l_finger_2_link_paraproximal_actuating_hinge','l_finger_2_link_paraproximal_bar','l_finger_2_link_proximal_actuating_hinge','l_finger_2_link_proximal_actuating_bar','l_finger_middle_link_0','l_finger_middle_link_1','l_finger_middle_link_2','l_finger_middle_link_3','l_finger_middle_link_paradistal_hinge','l_finger_middle_link_median_actuating_hinge','l_finger_middle_link_median_bar','l_finger_middle_link_paramedian_hinge','l_finger_middle_link_median_bar_underactuated','l_finger_middle_link_paraproximal_actuating_hinge','l_finger_middle_link_paraproximal_bar','l_finger_middle_link_proximal_actuating_hinge','l_finger_middle_link_proximal_actuating_bar','l_end_eff']; 
% value1 = {0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0};
% s = struct(field1,value1);

f(1).l_arm_shx = 0;
f(2).l_arm_ely= 0;
f(3).l_arm_elx = 0;
f(4).l_arm_wry = 0;
f(5).l_arm_wrx = 0;
f(6).l_arm_wry2 = 0;
f(7).l_palm_finger_1_joint = 0;
f(8).l_finger_1_joint_1 = 0;
f(9).l_finger_1_joint_2 = 0;
f(10).l_finger_1_joint_3 = 0;
f(11).l_finger_1_joint_paradistal_hinge = 0;
f(12).l_finger_1_joint_median_actuating_hinge= 0;
f(13).l_finger_1_joint_median_actuating_hinge_median_bar = 0;
f(14).l_finger_1_joint_paramedian_hinge = 0;
f(15).l_finger_1_joint_paramedian_hinge_median_bar_underactuated = 0;
f(16).l_finger_1_joint_paraproximal_actuating_hinge = 0;
f(17).l_finger_1_joint_paraproximal_bar = 0;
f(18).l_finger_1_joint_proximal_actuating_hinge = 0;
f(19).l_finger_1_joint_proximal_actuating_bar = 0;
f(20).l_palm_finger_2_joint = 0;
f(21).l_finger_2_joint_1 = 0;
f(22).l_finger_2_joint_2 = 0;
f(23).l_finger_2_joint_3 = 0;
f(24).l_finger_2_joint_paradistal_hinge = 0;
f(25).l_finger_2_joint_median_actuating_hinge = 0;
f(26).l_finger_2_joint_median_actuating_hinge_median_bar = 0;
f(27).l_finger_2_joint_paramedian_hinge = 0;
f(28).l_finger_2_joint_paramedian_hinge_median_bar_underactuated = 0;
f(29).l_finger_2_joint_paraproximal_actuating_hinge = 0;
f(30).l_finger_2_joint_paraproximal_bar = 0;
f(31).l_finger_2_joint_proximal_actuating_hinge = 0;
f(32).l_finger_2_joint_proximal_actuating_bar = 0;
f(33).l_palm_finger_middle_joint = 0;
f(34).l_finger_middle_joint_1 = 0;
f(35).l_finger_middle_joint_2 = 0;
f(36).l_finger_middle_joint_3 = 0;
f(37).l_finger_middle_joint_paradistal_hinge = 0;
f(38).l_finger_middle_joint_median_actuating_hinge = 0;
f(39).l_finger_middle_joint_median_actuating_hinge_median_bar = 0;
f(40).l_finger_middle_joint_paramedian_hinge = 0;
f(41).l_finger_middle_joint_paramedian_hinge_median_bar_underactuated = 0;
f(42).l_finger_middle_joint_paraproximal_actuating_hinge = 0;
f(43).l_finger_middle_joint_paraproximal_bar = 0;
f(44).l_finger_middle_joint_proximal_actuating_hinge = 0;
f(45).l_finger_middle_joint_proximal_actuating_bar = 0;

initialguess= f;


numTotalPoints = 30;

% Evaluate trajectory to create a vector of end-effector positions
eePositions = ppval(trajectory,linspace(0,trajectory.breaks(end),numTotalPoints));

for idx = 1:size(eePositions,2)
    tform = trvec2tform(eePositions(:,idx)');
    configSoln(idx,:) = ik('l_end_eff',tform,weights,initialguess);
    initialguess = configSoln(idx,:);
end


show(robot);

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

     
    