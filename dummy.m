clear all;

initial_BodyName = 'r_clav';
final_BodyName = 'r_palm';
[newSubtree] = subtree_generation(initial_BodyName,final_BodyName);
show(newSubtree);
newSubtree.DataFormat = "column";
newSubtree.Gravity = [0 0 -9.81];
wrench = [0 0 0.5 1 0 0.3];
fext = externalForce(newSubtree,'r_hand',wrench,homeConfiguration(newSubtree));
jointAccel = forwardDynamics(newSubtree,homeConfiguration(newSubtree),[],[],fext);