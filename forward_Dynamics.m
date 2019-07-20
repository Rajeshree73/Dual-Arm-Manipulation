% Forward Dynamics

initial_BodyName = 'r_clav';
final_BodyName = 'r_palm';
% call function subtree_generation
[newSubtree] = subtree_generation(initial_BodyName,final_BodyName);
show(newSubtree);
% Assign parameters for rigid body tree : newSubtree
newSubtree.DataFormat = "column";
newSubtree.Gravity = [0 0 -9.81];
% Torques and force applied to the body
wrench = [0 0 0.5 1 0 0.3];
% calculate external force due to gravity
fext = externalForce(newSubtree,'r_hand',wrench,homeConfiguration(newSubtree));
% Compute joint accelerations given joint torques and states
jointAccel = forwardDynamics(newSubtree,homeConfiguration(newSubtree),[],[],fext);

