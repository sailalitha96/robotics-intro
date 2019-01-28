%% this script is used to validate the forward kinematics .The ang is varible used to change the 
%% angle of each joint . Plot have been attached in the report . Independent jpint angles are plotted and trajecotry observed

lynxStart
nodes= [];
vel= [];

% this loop is ran to iterate the given joint angle between 0 to 90
% degrees, user can vary which q to change . 
for ang = 0: 0.05: 1.57
    q = [ 0, 0,0,ang,0,0];
    qdot= [ 0, 0,0,1,0,0] ;
    e_vel = FK_velocity(q, qdot);
    vel = [ vel ; e_vel];
    plotLynx(q)
    X = calculateFK_sol([q,0]);
    endpos= X(6, :);
    nodes =[ nodes;X(6, :)] ;
    text(X(6,1),X(6,2),X(6,3), '.');
    % angular quiver is scaled to make it visible. to plot quiver of
    % angular velocity uncomment below
    % quiver3(X(6,1) ,X(6,2),X(6,3), 40*e_vel(4, 1), 40*e_vel(5,1), 40*e_vel(6,1))
    % to plot quiver of linear velocity , uncomment below
    % quiver3(X(6,1) ,X(6,2),X(6,3), e_vel(1, 1), e_vel(2,1), e_vel(3,1)) %
   
    
  %% use the commented quiver to plot the velocity quiver . For plotting the angular velocity quiver plot the maginifed
  %% quiver 


    
     
end 

