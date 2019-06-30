% % 
% % % field1 = ['l_clav','l_scap','l_uarm','l_larm','l_ufarm','l_lfarm','l_hand','l_palm','l_finger_1_link_0','l_finger_1_link_1','l_finger_1_link_2','l_finger_1_link_3','l_finger_1_link_paradistal_hinge','l_finger_1_link_median_actuating_hinge','l_finger_1_link_median_bar','l_finger_1_link_paramedian_hinge','l_finger_1_link_median_bar_underactuated','l_finger_1_link_paraproximal_actuating_hinge','l_finger_1_link_paraproximal_bar','l_finger_1_link_proximal_actuating_hinge','l_finger_1_link_proximal_actuating_bar','l_finger_2_link_0','l_finger_2_link_1','l_finger_2_link_2','l_finger_2_link_3','l_finger_2_link_paradistal_hinge','l_finger_2_link_median_actuating_hinge','l_finger_2_link_median_bar','l_finger_2_link_paramedian_hinge','l_finger_2_link_median_bar_underactuated','l_finger_2_link_paraproximal_actuating_hinge','l_finger_2_link_paraproximal_bar','l_finger_2_link_proximal_actuating_hinge','l_finger_2_link_proximal_actuating_bar','l_finger_middle_link_0','l_finger_middle_link_1','l_finger_middle_link_2','l_finger_middle_link_3','l_finger_middle_link_paradistal_hinge','l_finger_middle_link_median_actuating_hinge','l_finger_middle_link_median_bar','l_finger_middle_link_paramedian_hinge','l_finger_middle_link_median_bar_underactuated','l_finger_middle_link_paraproximal_actuating_hinge','l_finger_middle_link_paraproximal_bar','l_finger_middle_link_proximal_actuating_hinge','l_finger_middle_link_proximal_actuating_bar','l_end_eff']; 
% % % value1 = {0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0};
% % % s = struct(field1,value1)
% % clear all;
% 
% %f(1).l_clav = 0;
% f(1).l_arm_shx = 0;
% f(2).l_arm_ely= 0;
% f(3).l_arm_elx = 0;
% f(4).l_arm_wry = 0;
% f(5).l_arm_wrx = 0;
% f(6).l_arm_wry2 = 0;
% f(7).l_robotiq_hand_joint = 0;
% f(8).l_robotiq_end_eff_virtual_joint = 0;
% f(9).l_palm_finger_1_joint = 0;
% f(10).l_palm_finger_2_joint = 0;
% f(11).l_palm_finger_middle_joint = 0;
% f(12).l_finger_1_joint_proximal_actuating_hinge = 0;
% f(13).l_finger_1_joint_paraproximal_actuating_hinge = 0;
% f(14).l_finger_1_joint_1 = 0;
% f(15).l_finger_1_joint_median_actuating_hinge= 0;
% f(16).l_finger_1_joint_paramedian_hinge = 0;
% f(17).l_finger_1_joint_2 = 0;
% f(18).l_finger_1_joint_paradistal_hinge = 0;
% f(19).l_finger_1_joint_3 = 0;
% f(20).l_finger_1_link_paradistal_hinge_link_3_couple = 0;
% f(21).l_finger_1_link_median_bar_link_3_couple = 0;
% f(22).l_finger_1_link_paramedian_bar_paradistal_hinge_couple = 0;
% f(23).l_finger_1_joint_median_actuating_hinge_median_bar = 0;
% f(24).l_finger_1_link_proximal_actuating_bar_median_actuating_hinge_couple = 0; %#ok<LNGNM>
% f(25).l_finger_1_joint_paramedian_hinge_median_bar_underactuated = 0;
% f(26).l_finger_1_link_paraproximal_bar_paramedian_hinge_couple = 0;
% f(27).l_finger_1_joint_paraproximal_bar = 0;
% f(28).l_finger_1_joint_proximal_actuating_bar = 0;
% f(29).l_finger_2_joint_proximal_actuating_hinge = 0;
% f(30).l_finger_2_joint_paraproximal_actuating_hinge = 0;
% f(31).l_finger_2_joint_1 = 0;
% f(32).l_finger_2_joint_median_actuating_hinge = 0;
% f(33).l_finger_2_joint_paramedian_hinge = 0;
% f(34).l_finger_2_joint_2 = 0;
% f(35).l_finger_2_joint_paradistal_hinge = 0;
% f(36).l_finger_2_joint_3 = 0;
% f(37).l_finger_2_link_median_bar_link_3_couple = 0;
% f(38).l_finger_2_link_paradistal_hinge_link_3_couple = 0;
% f(39).l_finger_2_link_paramedian_bar_paradistal_hinge_couple = 0;
% % f(40).l_finger_2_joint_median_actuating_hinge_median_bar = 0;
% % f(41).l_finger_2_link_proximal_actuating_bar_median_actuating_hinge_couple = 0; %#ok<LNGNM>
% % f(42).l_finger_2_joint_paramedian_hinge_median_bar_underactuated = 0;
% % f(43).l_finger_2_link_paraproximal_bar_paramedian_hinge_couple = 0;
% % f(44).l_finger_2_joint_paraproximal_bar = 0;
% % f(45).l_finger_2_joint_proximal_actuating_bar = 0;
% % % f(46).l_finger_middle_joint_proximal_actuating_hinge = 0;
% % % f(47).l_finger_middle_joint_paraproximal_actuating_hinge = 0;
% % % f(48).l_finger_middle_joint_1 = 0;
% % % f(49).l_finger_middle_joint_median_actuating_hinge = 0;
% % % l_finger_middle_joint_paramedian_hinge
% % % l_finger_middle_joint_2
% % % l_finger_middle_joint_paradistal_hinge
% % % l_finger_middle_joint_3
% % % l_finger_middle_link_median_bar_link_3_couple
% % % l_finger_middle_link_paradistal_hinge_link_3_couple
% % % l_finger_middle_link_paramedian_bar_paradistal_hinge_couple
% % % l_finger_middle_joint_median_actuating_hinge_median_bar
% % % l_finger_middle_link_proximal_actuating_bar_median_actuating_hinge_couple
% % % l_finger_middle_joint_paramedian_hinge_median_bar_underactuated
% % % l_finger_middle_link_paraproximal_bar_paramedian_hinge_couple
% % % l_finger_middle_joint_paraproximal_bar
% % % l_finger_middle_joint_proximal_actuating_bar
clear all;
f(1).JointPosition = {'l_arm_shx';'l_arm_ely';'l_arm_elx';'l_arm_wry';'l_arm_wrx';'l_arm_wry2';'l_palm_finger_1_joint';'l_finger_1_joint_1';'l_finger_1_joint_2';'l_finger_1_joint_3';'l_finger_1_joint_paradistal_hinge';'l_finger_1_joint_median_actuating_hinge';'l_finger_1_joint_median_actuating_hinge_median_bar';'l_finger_1_joint_paramedian_hinge';'l_finger_1_joint_paramedian_hinge_median_bar_underactuated';'l_finger_1_joint_paraproximal_actuating_hinge';'l_finger_1_joint_paraproximal_bar';'l_finger_1_joint_proximal_actuating_hinge';'l_finger_1_joint_proximal_actuating_bar';'l_palm_finger_2_joint';'l_finger_2_joint_1';'l_finger_2_joint_2';'l_finger_2_joint_3';'l_finger_2_joint_paradistal_hinge';'l_finger_2_joint_median_actuating_hinge';'l_finger_2_joint_median_actuating_hinge_median_bar';'l_finger_2_joint_paramedian_hinge';'l_finger_2_joint_paramedian_hinge_median_bar_underactuated';'l_finger_2_joint_paraproximal_actuating_hinge';'l_finger_2_joint_paraproximal_bar';'l_finger_2_joint_proximal_actuating_hinge';'l_finger_2_joint_proximal_actuating_bar';'l_palm_finger_middle_joint';'l_finger_middle_joint_1';'l_finger_middle_joint_2';'l_finger_middle_joint_3';'l_finger_middle_joint_paradistal_hinge';'l_finger_middle_joint_median_actuating_hinge';'l_finger_middle_joint_median_actuating_hinge_median_bar';'l_finger_middle_joint_paramedian_hinge';'l_finger_middle_joint_paramedian_hinge_median_bar_underactuated';'l_finger_middle_joint_paraproximal_actuating_hinge';'l_finger_middle_joint_paraproximal_bar';'l_finger_middle_joint_proximal_actuating_hinge';'l_finger_middle_joint_proximal_actuating_bar'};   
f(2).JointValue = {0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0};
s = f;


