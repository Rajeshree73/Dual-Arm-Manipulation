
clear all;
% f(1).JointPosition = {'l_arm_shx';'l_arm_ely';'l_arm_elx';'l_arm_wry';'l_arm_wrx';'l_arm_wry2';'l_palm_finger_1_joint';'l_finger_1_joint_1';'l_finger_1_joint_2';'l_finger_1_joint_3';'l_finger_1_joint_paradistal_hinge';'l_finger_1_joint_median_actuating_hinge';'l_finger_1_joint_median_actuating_hinge_median_bar';'l_finger_1_joint_paramedian_hinge';'l_finger_1_joint_paramedian_hinge_median_bar_underactuated';'l_finger_1_joint_paraproximal_actuating_hinge';'l_finger_1_joint_paraproximal_bar';'l_finger_1_joint_proximal_actuating_hinge';'l_finger_1_joint_proximal_actuating_bar';'l_palm_finger_2_joint';'l_finger_2_joint_1';'l_finger_2_joint_2';'l_finger_2_joint_3';'l_finger_2_joint_paradistal_hinge';'l_finger_2_joint_median_actuating_hinge';'l_finger_2_joint_median_actuating_hinge_median_bar';'l_finger_2_joint_paramedian_hinge';'l_finger_2_joint_paramedian_hinge_median_bar_underactuated';'l_finger_2_joint_paraproximal_actuating_hinge';'l_finger_2_joint_paraproximal_bar';'l_finger_2_joint_proximal_actuating_hinge';'l_finger_2_joint_proximal_actuating_bar';'l_palm_finger_middle_joint';'l_finger_middle_joint_1';'l_finger_middle_joint_2';'l_finger_middle_joint_3';'l_finger_middle_joint_paradistal_hinge';'l_finger_middle_joint_median_actuating_hinge';'l_finger_middle_joint_median_actuating_hinge_median_bar';'l_finger_middle_joint_paramedian_hinge';'l_finger_middle_joint_paramedian_hinge_median_bar_underactuated';'l_finger_middle_joint_paraproximal_actuating_hinge';'l_finger_middle_joint_paraproximal_bar';'l_finger_middle_joint_proximal_actuating_hinge';'l_finger_middle_joint_proximal_actuating_bar'};   
% f(2).JointValue = {0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0};
% s =
% struct('JointPosition',{'l_arm_shx','l_arm_ely','l_arm_elx','l_arm_wry','l_arm_wrx','l_arm_wry2','l_palm_finger_1_joint','l_finger_1_joint_1','l_finger_1_joint_2','l_finger_1_joint_3','l_finger_1_joint_paradistal_hinge','l_finger_1_joint_median_actuating_hinge','l_finger_1_joint_median_actuating_hinge_median_bar','l_finger_1_joint_paramedian_hinge','l_finger_1_joint_paramedian_hinge_median_bar_underactuated','l_finger_1_joint_paraproximal_actuating_hinge','l_finger_1_joint_paraproximal_bar','l_finger_1_joint_proximal_actuating_hinge','l_finger_1_joint_proximal_actuating_bar','l_palm_finger_2_joint','l_finger_2_joint_1','l_finger_2_joint_2','l_finger_2_joint_3','l_finger_2_joint_paradistal_hinge','l_finger_2_joint_median_actuating_hinge','l_finger_2_joint_median_actuating_hinge_median_bar','l_finger_2_joint_paramedian_hinge','l_finger_2_joint_paramedian_hinge_median_bar_underactuated','l_finger_2_joint_paraproximal_actuating_hinge','l_finger_2_joint_paraproximal_bar','l_finger_2_joint_proximal_actuating_hinge','l_finger_2_joint_proximal_actuating_bar','l_palm_finger_middle_joint','l_finger_middle_joint_1','l_finger_middle_joint_2','l_finger_middle_joint_3','l_finger_middle_joint_paradistal_hinge','l_finger_middle_joint_median_actuating_hinge','l_finger_middle_joint_median_actuating_hinge_median_bar','l_finger_middle_joint_paramedian_hinge','l_finger_middle_joint_paramedian_hinge_median_bar_underactuated','l_finger_middle_joint_paraproximal_actuating_hinge','l_finger_middle_joint_paraproximal_bar','l_finger_middle_joint_proximal_actuating_hinge','l_finger_middle_joint_proximal_actuating_bar'},'JointValue',{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0});
% 

addpath(genpath(strcat(pwd,'/home/rajeshree/catkin_ws/src/manipulation')));
robot = createRigidBodyTree;
axes = show(robot);
axes.CameraPositionMode = 'auto';
base = robot.BaseName; 

%% Make a subtree from utorso 
newSubtree = subtree(robot,'utorso');
a = removeBody(newSubtree,'l_palm');
b = removeBody(newSubtree,'r_clav');
c = removeBody(newSubtree,'l_situational_awareness_camera_link');
d = removeBody(newSubtree,'head');
c = removeBody(newSubtree,'r_situational_awareness_camera_link');
newJoint = robotics.Joint('fixed');
replaceJoint(newSubtree,'utorso',newJoint);






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

