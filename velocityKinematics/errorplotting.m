%% This script runs the plot for error in z and radius . This is written only for flat circles 

% lynxStart % we commented lynxStart to plot the error graph 
q= [ 0 ,0 ,0 ,0 ,0];

% specify the radius needed
R= 80;
x_origin= 263.5250- R;
y_origin = 0.0000 ;
z_origin = 222.2500;

jointpoints= [];
syms t 
pos_x= sin(t);
pos_y = cos(t);
z= 0 ;
nodes = [];
cumerror= [] ;
cumradius= [];
i = [] ; 

X = calculateFK_sol([q,0]);
zprev= X(6, 3);

% you can see the omega is same as the time step given. The loop is ran to
% plot the circle in the given plane. 
for t = 0: 0.01: 3*6.28
  % each tie step is recorded
    i = [i ; t ]; 
 % equations for the velocity of a circle are hardcoded    
    vy = R*cos(t);
    vx = R* -sin(t);
    vz= 0 ; 
    velocity = [ vx; vy ; vz] ;
  % e_vel is constantly updated and in turn updates q 
    e_vel = [0.01* velocity(1); 0.01*velocity(2); 0.01* velocity(3); 0 ;  0 ; 0.01];
    qdot = IK_velocity(q,e_vel);
    q = q + qdot'; 
    jointpoints= [ jointpoints ; q];
% Uncomment below if wanting to plot while running the loop.
%     plotLynx([q,0]); 

% calculating the z error and radius error. 
    X = calculateFK_sol([q,0]);
    nodes =[ nodes;X(6, :)];
    z= X(6,3);
    radx= X(6,1);
    rady= X(6,2);
    radius = sqrt( (x_origin - radx)^2 + (y_origin- rady)^2);
    raderr= radius - R;
    % each error i radius is recorded.
    cumradius = [ cumradius ; raderr] ; 
    error = z- zprev ; 
    % each error in z is recorded and stored.
    cumerror = [ cumerror ; error]; 
% quiver for linear and angular velocities    
%     quiver3(X(6,1) ,X(6,2),X(6,3),1000*e_vel(1, 1),1000*e_vel(2,1),1000*e_vel(3,1)) 
%     quiver3(X(6,1) ,X(6,2),X(6,3),1000*e_vel(4, 1),1000*e_vel(5,1),1000*e_vel(6,1)) 
    text(X(6,1),X(6,2),X(6,3),'.');
    
     
end 

% provides maximum and minimum points observed during the trajectory. 
x_max= max(nodes(:,1));
y_max = max(nodes(:,2));
z_max = max(nodes(:,3));
maxpositions =[ x_max y_max z_max] 
x_min = min(nodes(:,1));
y_min = min(nodes(:,2));
z_min = min(nodes(:,3));
minpositions = [ x_min y_min z_min] 

% plotting errors , this plot both the graphs on the same plot
plot(i ,cumerror , '-k')
hold on 
plot(i , cumradius, '-g')
    
