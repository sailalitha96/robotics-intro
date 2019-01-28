function qdot = IK_velocity(q, e_vel)
% Input: q - 1 x 6 vector of joint inputs [q1,q2,q3,q4,q5,q6]
%        e_vel - 6 x 1 vector of end effector velocities, where
%                    e_vel(1:3) are the linear velocity
%                    e_vel(4:6) are the angular velocity

% Output: qdot - 1 x 6 vector of joint velocities [q1dot,q2dot,q3dot,q4dot,q5dot,q6dot]


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Lynx ADL5 constants in mm
d1 = 76.2; % base height (table to center of joint 2)
a2 = 146.05; % shoulder to elbow length
a3 = 187.325; %elbow to wrist length
d5 = 76.2; %wrist to base of gripper
lg = 28.575; %length of gripper

%% Your code here
q1= q(1);
q2= q(2);
q3= q(3);
q4= q(4); 
% the jacobian has been hard coded 
Jacobian_sim =  [ 38*sin(q2 - q1 + q3 + q4) - 93*sin(q1 + q2 + q3) - 73*cos(q1 - q2) - 38*sin(q1 + q2 + q3 + q4) + 73*cos(q1 + q2) + 93*sin(q2 - q1 + q3), 73*cos(q1 - 1.0*q2) - 38*sin(q2 - 1.0*q1 + q3 + q4) - 93*sin(q1 + q2 + q3) - 38*sin(q1 + q2 + q3 + q4) + 73*cos(q1 + q2) - 93*sin(q2 - 1.0*q1 + q3) ,-93*sin(q1 + q2 + q3) - 38*sin(q2 - q1 + q3 + q4) - 38*sin(q1 + q2 + q3 + q4) - 93*sin(q2 - q1 + q3) , -38*sin(q2 - q1 + q3 + q4) - 38*sin(q1 + q2 + q3 + q4) , 0 ; 
93*cos(q1 + q2 + q3) + 38*cos(q2 - q1 + q3 + q4) - 73*sin(q1 - q2) + 38*cos(q1 + q2 + q3 + q4) + 73*sin(q1 + q2) + 93*cos(q2 - q1 + q3), 93*cos(q1 + q2 + q3) - 38*cos(q2 - q1 + q3 + q4) + 73*sin(q1 - q2) + 38*cos(q1 + q2 + q3 + q4) + 73*sin(q1 + q2) - 93*cos(q2 - q1 + q3), 93*cos(q1 + q2 + q3) - 38*cos(q2 - 1.0*q1 + q3 + q4) + 38*cos(q1 + q2 + q3 + q4) - 93*cos(q2 - 1.0*q1 + q3),38*cos(q1 + q2 + q3 + q4)- 38*cos(q2 - 1.0*q1 + q3 + q4), 0 ; 
0 , - 76.2*cos(q2 + q3 + q4) - 187.325*cos(q2 + q3) - 146.05*sin(q2) ,  - 76.2*cos(q2 + q3 + q4) - 187.325*cos(q2 + q3), -76.2*cos(q2 + q3 + q4), 0 ;
 0 , -sin(q1), -sin(q1),  -sin(q1), 0.5*cos(q2 - 1.0*q1 + q3 + q4) + 0.5*cos(q1 + q2 + q3 + q4);
 0 , cos(q1) , cos(q1) , cos(q1) , 0.5*sin(q1 + q2 + q3 + q4) - 0.5*sin(q2 - 1.0*q1 + q3 + q4) ;
 1, 0 , 0 , 0 ,- 1.0*sin(q2 + q3 + q4)];
Jacobian_sim = round(Jacobian_sim);
jrank = rank(Jacobian_sim);
if( jrank == 5)
    jmatrix= Jacobian_sim; 
else 
    disp(" this will be a singularity position")
    jmatrix= Jacobian_sim; 
end 
% jmatrix = jmatrix(1:3,:);
% e_vel = e_vel(1:3);
% we commented this because we want to compare with the whole of jmatrix.
% the rank test is performed but not considered , for reasons explained in
% the report 
PsuedoJ = pinv(jmatrix);
checkmatrix= [jmatrix, e_vel] ;
rank(jmatrix);
checkrank= rank(checkmatrix);
if (checkrank== rank(jmatrix))
    qdot = PsuedoJ* e_vel;
else 
    qdot = PsuedoJ* e_vel;
        %  disp(" this velocity is not permissible")
        %  qdot = [ 0 0 0 0 0 ] 
% the above portion has been been commented out since there are points in
% the trajectory it cant achieve . It works for stand alone points, but to
% plana trajecotry we have commented it out.
end 

end