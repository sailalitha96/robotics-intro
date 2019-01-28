%% THIS CODE PROVIDES THE output of a tilted circle. 
lynxStart
q= [ pi/2,0,0 ,0 ,0];  % provide start position

nodes = [];
% modify radius
R = 100; 
% we are fixing the tilt to be 45 degrees and calculate the omega vector
% accordingly
theta = pi/4 ; 
% we have written the equations for a tilted circle and iterate for phi and
% updated our qdot and q accordingly to get the desiered trajectory. 
for phi = 0: 0.03 : 6.48
 % hardcoded the velocity equations for the tilted circle 
    vx = R*-1*(cos(theta)*sin(phi) + sin(theta)*sin(phi));
    vy = R *( cos(theta)*cos(phi) - sin(theta)* sin(phi));
    vz= R* cos(phi); 
    velocity = [ vx; vy ; vz ] ; 
 % updating e_vel and and q for each iteration
    X= calculateFK_sol(q);
    e_vel =[0.03*vx; 0.03* vy; 0.03*vz; 0.03 *0.707 ; 0.03 *-0.707  ; 0.03* 0.707];
    qdot = IK_velocity(q,e_vel);
    q = q + qdot'; 
    
    plotLynx([q,0]);
    X = calculateFK_sol([q,0]);
    nodes =[ nodes;X(6, :)];
% to plot quiver  uncomment below 
%     quiver3(X(6,1),X(6,2),X(6,3)+lg,e_vel(1, 1), e_vel(2, 1), e_vel(3,1), ' r')
    text(X(6,1),X(6,2),X(6,3),'.');
    
     
end 

x_max = max(nodes(:,1));
y_max = max(nodes(:,2));
z_max = max(nodes(:,3));
maxpositions = [ x_max y_max z_max];
x_min = min(nodes(:,1));
y_min = min(nodes(:,2));
z_min = min(nodes(:,3));
minpositions= [ x_min y_min z_min];