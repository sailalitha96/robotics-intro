%% THIS CODE PROVIDES THE output of a tilted circle. 
lynxStart
q= [ pi/2,0,0 ,0 ,0];  % provide start position
e_vel = FK_velocity(q, q_dot); % getting the inital end effector velocity 
nodes = [];
R = 100; % radius needed
theta = pi/4 ; 

for phi = 0: 0.03 : 6.48
   
    vx = R*-1*(cos(theta)*sin(phi) + sin(theta)*sin(phi));
    vy = R *( cos(theta)*cos(phi) - sin(theta)* sin(phi));
    vz= R* cos(phi); 
    velocity = [ vx; vy ; vz ] ; 
    
    X= calculateFK_sol(q);
    e_vel =[0.03*vx; 0.03* vy; 0.03*vz; 0.03 *0.707 ; 0.03 *-0.707  ; 0.03* 0.707];
    qdot = IK_velocity(q,e_vel);
    q = q + qdot'; 
    
    plotLynx([q,0]);
    X = calculateFK_sol([q,0]);
    nodes =[ nodes;X(6, :)];
%     quiver3(X(6,1),X(6,2),X(6,3)+lg,e_vel(1, 1), e_vel(2, 1), e_vel(3,1), ' r')% if quiver needed uncomment 
    text(X(6,1),X(6,2),X(6,3),'.');
    
     
end 

x_max = max(nodes(:,1));
y_max = max(nodes(:,2));
z_max = max(nodes(:,3));
maxpositions = [ x_max y_max z_max]
x_min = min(nodes(:,1));
y_min = min(nodes(:,2));
z_min = min(nodes(:,3));
minpositions= [ x_min y_min z_min]