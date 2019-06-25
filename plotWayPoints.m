% Create the set of waypoints to serve as a trajectory
function plotWayPoints(wayPoints)

for i = 1:size(wayPoints,1);
    plotSpheres(0.02,wayPoints(i,:));
end
end