close all;
clc;
% lenghths of the two arms
l1 = 10;
l2 = 8;

theta1 = 0:0.1:pi/2;
theta2 = 0:0.1:pi;

% generate grid
[Theta1,Theta2] = meshgrid(theta1,theta2);

% compute x and y co-ordinates
X = l1 * cos(Theta1) + l2 * cos(Theta1 + Theta2);
Y = l1 * sin(Theta1) + l2 * sin(Theta1 + Theta2);

% create a data-set

data1 = [X(:) Y(:) Theta1(:)];
data2 = [X(:) Y(:) Theta2(:)];

plot(X(:),Y(:),'r');
axis equal;

