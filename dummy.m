% Import and load the rigid body tree
initial_BodyName = 'r_clav';
final_BodyName = 'r_palm';
[newSubtree] = subtree_generation(initial_BodyName,final_BodyName);

newSubtree.DataFormat = 'row';
newSubtree.Gravity = [0, 0, -9.80665];
show(newSubtree);
% Create the set of desired wayPoints trajectory
qWaypoints = [-0.7, -1.0, 0.55,-0.5, -0.9, 0.6,-0.3;-0.8, 0.75,-0.1,-0.7,0.7,0.1,-0.6;0.75,0.3, -0.5, 0.6,0.5, -0.45, 0.65];

% qWaypoints = [0,0,0,0,0,0,0;1.38103270582186,2.09420000000000,-1.86997455286055,1.78667692528909,1.34927161002807,-0.156819546182919,-1.92929157299663;1.61943622374369,2.09420000000000,-1.75104236580808,1.59968158260377,1.38094537808453,-0.156647974766754,-1.91487924205474];
% plotWayPoints(wayPoints);
% 
% % Create a smooth curve from the wayPoints to track the trajectory using a curve fitting toolbox function cscvn
% desired_trajectory = cscvn(wayPoints');
% % hold off;
% % Plot the trajectory spline joining the waypoints
% hold on
% fnplt(desired_trajectory,'r',2);
tWaypoints = [0,1,2];
cdt = 0.001; % planned control stepsize
tt =  0:cdt:5;
% Generate desired motion trajectory for each joint
% joint_traj_generation generates joint trajectories from given time and joint configuration waypoints
% the trajectories are generated using pchip
[qDesired, qdotDesired, qddotDesired, tt] =  joint_traj_generation( tWaypoints, qWaypoints, tt );

% Pre-compute feed-forward torques that ideally would realize the desired motion using Inverse Dynamics 
n = size(qDesired,1);
tauFeedForward = zeros(n,7);
for i = 1:n
    tauFeedForward(i,:) = inverseDynamics(newSubtree, qDesired(i,:), qdotDesired(i,:), qddotDesired(i,:));
end


% weights = [0.3,0.8,0.6, 0.6,0.3,0.2,0.1];
% Kp = 100*weights;
% Kd = 2* weights;
% 
% once = 1;
% 
% feedForwardTorque = zeros(n, 7);
% pdTorque = zeros(n, 7);
% timePoints = zeros(n,1);
% Q = zeros(n,7);
% QDesired = zeros(n,7);


%axis([-0.1 0.4 -0.35 0.35 0 0.35]);
for i = 1:size(tauFeedForward(i,:))
    show(newSubtree,tauFeedForward(i,:));
    pause(0.01)
end
hold off




