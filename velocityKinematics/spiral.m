%% run this script for a spiral 
lynxStart
% provide start position
q= [ -pi/6 ,0 ,0 ,0 ,0]; 
q_dot= [1,0,0,2,0] ; 
e_vel = FK_velocity(q, q_dot); % getting the inital end effector velocity 

jointpoints = [];
syms t 
nodes = [];

% provide R and a for varying the radius and heights of the spiral
R = 50 ;
a= 30 ;
% this loop has been run for 2 loops 
for t = 0: 0.01: 2*6.48
    
% hardcoded the velocity equations for spiral . 
    vy = R*cos(t);
    vx = R*-sin(t) ;
    vz= a / (2* pi); 
    X= calculateFK_sol(q);
   % updating e_Vel and q for each iteration 
    e_vel = [ 0.01*vx ; 0.01*vy ; 0.01*vz ; 0 ; 0 ; 0.01];
    qdot = IK_velocity(q,e_vel);
    q = q + qdot'; 
    jointpoints= [ jointpoints ; q];
    %Uncomment below if plotting while running is needed
    %plotLynx([q,0]);       
    X = calculateFK_sol([q,0]);
    nodes =[ nodes;X(6, :)];
% uncomment to plot the quiver plot for linear velocity
%     quiver3(X(6,1),X(6,2),X(6,3) +lg,e_vel(1, 1), e_vel(2, 1), e_vel(3,1), ' r') 
    text(X(6,1),X(6,2),X(6,3),'.');
   
end 

plot(jointpoints)
hold on 
plotLynx(q)

% calculate minimum and maximum positions 
x_max= max(nodes(:,1));
y_max = max(nodes(:,2));
z_max = max(nodes(:,3));
maxpositions = [ x_max, y_max, z_max];
x_min = min(nodes(:,1));
y_min = min(nodes(:,2));
z_min = min(nodes(:,3));
minpositions= [ x_min,y_min, z_min];