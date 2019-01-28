%% this script to run to obtain the trajectory for constant qdot 

lynxStart
 % getting the inital starting point
q= [ 0 ,0 ,pi/6,0 ,0 ];  

nodes= [];
syms t 
pos_x= sin(t);
pos_y = cos(t);
z= 0 ;
jointspoints= [];
R= 20;

% this loop is run to update q with time and plot q accordingly. 
for t = 0: 0.05: 50
    
% constant linear velocities and qdot are given and e_vel is calculated.
    vy = 9; 
    vz = 9;
    vx= 9 ; 
    e_vel= [ R*vx ; R*vy ;  R*vx ; 0 ; 0 ; 0.05 ];
    qdot = [ 1; 1; 1; 1; 1];
    q = q + qdot'; 
    jointspoints= [ jointspoints ; q ] ;
    
 % plotting is done in the loop 
    plotLynx([q,0])
    X = calculateFK_sol([q,0]);
    endpos= X(6, :);
    nodes =[ nodes;X(6, :)] ;
    % uncomment below line if quiver plot needed
    %quiver3(X(6,1) ,X(6,2),X(6,3), 0.01*e_vel(1, 1), 0.01*e_vel(2,1),0.01*e_vel(3,1)) 
    text(X(6,1),X(6,2),X(6,3), '.');
   
    
     
end 

x_max= max(nodes(:,1));
y_max = max(nodes(:,2));
z_max= max(nodes(:,3));
maxpositions = [ x_max , y_max , z_max]; 
x_min =min(nodes(:,1));
y_min= min(nodes(:,2));
z_min = min(nodes(:,3));
minpositions= [x_min , y_min, z_min]; 
