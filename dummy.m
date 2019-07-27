% Import and load the rigid body tree
initial_BodyName = 'r_clav';
final_BodyName = 'r_palm';
[newSubtree] = subtree_generation(initial_BodyName,final_BodyName);
homeConfig = newSubtree.homeConfiguration;
newSubtree.DataFormat = 'column';
newSubtree.Gravity = [0, 0, -9.80665];
show(newSubtree);
% Create the set of desired wayPoints trajectory
% wayPoints = [-0.5 -0.7 0.55;-0.45 -0.7 0.6;-0.1 -0.7 0.7;0.1 -0.7 0.75;0.3 -0.7 0.6;0.5 -0.7 0.65;0.7 -0.7 0.5];
 wayPoints = [-0.5 -0.7 0.55];
plotWayPoints(wayPoints);
trajectory = cscvn(wayPoints');

% Plot trajectory spline and waypoints
hold on
fnplt(trajectory,'r',2);

jacobian = geometricJacobian(newSubtree,homeConfiguration(newSubtree),'r_hand');

% Define PID constants
Kp1 = 8; 
Kp2 = 8;
Kp3 = 8;
Kp4 = 8;
Kp5 = 8;
Kp6 = 8;
Kp7 = 8;
Kd1 = 0.100000000000000;
Kd2 = 0.100000000000000;
Kd3 = 0.100000000000000;
Kd4 = 0.100000000000000;
Kd5 = 0.100000000000000;
Kd6 = 0.100000000000000;
Kd7 = 0.100000000000000;

Sf = [1 0 0 0;0 0 0 0;0 1 0 0;0 0 1 0; 0 0 0 1;0 0 0 0];
Sv = eye(6);
Sv(2,2) = 0;


%% Perform the ODE to find out v so that we can put it in some another equations.

