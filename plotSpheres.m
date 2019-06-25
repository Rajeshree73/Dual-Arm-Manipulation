% Create a function to plot points for visualizing a sphere

function plotSpheres(size,pointPosition)

% create a point for visualizing a sphere
[X,Y,Z] = sphere(20);

% Translate to the specified position
X = size*X+ pointPosition(1);
Y = size*Y+ pointPosition(2);
Z = size*Z+ pointPosition(3);

% Add the sphere to the figure and configuring the lighting
s = patch(surf2patch(X,Y,Z));
s.FaceColor = 'blue';
s.FaceLighting = 'gouraud';
s.EdgeAlpha = 0;

% Move the light so that the side of the cup is illuminated
lightObj = findobj(gca,'Type','Light');
lightObj.Position = [1,1,1];

end
