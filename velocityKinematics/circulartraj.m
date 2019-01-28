%% this script plot the circle .Wait for the plot ,the script is written to collect points and point at the end 

lynxStart
q= [ 0 ,0 ,pi/3,0 ,0]; % provide q start 
a3 = 187.325; %elbow to wrist length
d5 = 76.2; %wrist to base of gripper
lg = 28.575; %length of gripper
% change R 
R = 200;
jointpoints= [];
syms t 
pos_x= sin(t);
pos_y = cos(t);
z= 0 ;
nodes = [];


% you can see the omega is same as the time step given. The loop is ran to
% plot the circle in the given plane. The rate at which t is changed is the
% rate at which the omega is changed.
for t = 0: 0.005: 3*6.28

 % the differentiated equations of velocity as hardcoded to avoid using syms .    
    vy = R*cos(t);
    vx = R* -sin(t);
    vz= 0 ; 
    velocity = [ vx; vy ; vz] ;
 % end velocities are calculated and joint positions are updated each time using the IK_velocity.
    e_vel = [0.005* velocity(1); 0.005*velocity(2); 0.005* velocity(3); 0 ;  0 ; 0.005]; 
    qdot = IK_velocity(q,e_vel);
    q = q + qdot'; 
    jointpoints= [ jointpoints ; q];
% Uncomment below to plot while running the loop 
%     plotLynx([q,0]); 
    X = calculateFK_sol([q,0]);
    nodes =[ nodes;X(6, :)];
% Uncomment below to plot quiver while running the loop 
%     quiver3(X(6,1) ,X(6,2),X(6,3),1000*e_vel(1, 1), 1000*e_vel(2,1), 1000*e_vel(3,1))
    text(X(6,1),X(6,2),X(6,3),'.');
    
     
end 
plot(jointpoints) % plot all the points at once 
hold on 
plotLynx(q)


% provides maximum and minimum points observed during the trajectory. 
x_max= max(nodes(:,1));
y_max = max(nodes(:,2));
z_max = max(nodes(:,3));
maxpositions =[ x_max y_max z_max] ;
x_min = min(nodes(:,1));
y_min = min(nodes(:,2));
z_min = min(nodes(:,3));
minpositions = [ x_min y_min z_min]; 


    
