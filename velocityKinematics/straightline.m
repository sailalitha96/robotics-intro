%% run this script for plotting a straight line
lynxStart
% provide starting position
q= [ 0,pi/4 ,0 ,0 ,0 ]; 
nodes= [];
syms t 
% provide x y z velocities 
    vx = 2;
    vy = 10;
    vz= 10 ; 
for t = 0: 0.05: 10
    e_vel= [ vx ; vy ;  vz ; 0 ; 0 ; 0 ];% angular velocity is 0 since straight line
    qdot = IK_velocity(q,e_vel);
    q = q + qdot'* 0.05;  % time stepping and updating 
    plotLynx([q,0])
    X = calculateFK_sol([q,0]);
    nodes =[ nodes;X(6, :)] ;
    text(X(6,1),X(6,2),X(6,3), '.');
    % uncomment below if quiver plot of linest velcoity is needed. 
    %quiver3(X(6,1) ,X(6,2),X(6,3),10*e_vel(1, 1),10*e_vel(2,1),10* e_vel(3,1))
end 

% calculating the start and end points , slopes for quantitative evaluation
x_max= max(nodes(:,1));
y_max = max(nodes(:,2));
z_max= max(nodes(:,3));
maxpositions= [ x_max , y_max , z_max] ;
x_min =min(nodes(:,1));
y_min= min(nodes(:,2));
z_min = min(nodes(:,3));
minpositions = [x_min , y_min, z_min] ;
slopexz = (x_max - x_min )/( z_max- z_min );
slopexy= (x_max - x_min )/( y_max- y_min );
slopezy= (y_max - y_min )/( z_max- z_min );