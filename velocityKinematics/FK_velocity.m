function  e_vel = FK_velocity(q, qdot)
% Input: q - 1 x 6 vector of joint inputs [q1,q2,q3,q4,q5,q6]
%        qdot - 1 x 6 vector of joint velocities [q1dot,q2dot,q3dot,q4dot,q5dot,q6dot]

% Outputs:  e_vel - 6 x 1 vector of end effector velocities, where
%                    e_vel(1:3) are the linear velocity
%                    e_vel(4:6) are the angular velocity

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
q5= q(5);
% hardcoded jacobian 
Jacobian_sim =  [ 38*sin(q2 - q1 + q3 + q4) - 93*sin(q1 + q2 + q3) - 73*cos(q1 - q2) - 38*sin(q1 + q2 + q3 + q4) + 73*cos(q1 + q2) + 93*sin(q2 - q1 + q3), 73*cos(q1 - 1.0*q2) - 38*sin(q2 - 1.0*q1 + q3 + q4) - 93*sin(q1 + q2 + q3) - 38*sin(q1 + q2 + q3 + q4) + 73*cos(q1 + q2) - 93*sin(q2 - 1.0*q1 + q3) ,-93*sin(q1 + q2 + q3) - 38*sin(q2 - q1 + q3 + q4) - 38*sin(q1 + q2 + q3 + q4) - 93*sin(q2 - q1 + q3) , -38*sin(q2 - q1 + q3 + q4) - 38*sin(q1 + q2 + q3 + q4) , 0 ; 
93*cos(q1 + q2 + q3) + 38*cos(q2 - q1 + q3 + q4) - 73*sin(q1 - q2) + 38*cos(q1 + q2 + q3 + q4) + 73*sin(q1 + q2) + 93*cos(q2 - q1 + q3), 93*cos(q1 + q2 + q3) - 38*cos(q2 - q1 + q3 + q4) + 73*sin(q1 - q2) + 38*cos(q1 + q2 + q3 + q4) + 73*sin(q1 + q2) - 93*cos(q2 - q1 + q3), 93*cos(q1 + q2 + q3) - 38*cos(q2 - 1.0*q1 + q3 + q4) + 38*cos(q1 + q2 + q3 + q4) - 93*cos(q2 - 1.0*q1 + q3),38*cos(q1 + q2 + q3 + q4)- 38*cos(q2 - 1.0*q1 + q3 + q4), 0 ; 
0 , - 76.2*cos(q2 + q3 + q4) - 187.325*cos(q2 + q3) - 146.05*sin(q2) ,  - 76.2*cos(q2 + q3 + q4) - 187.325*cos(q2 + q3), -76.2*cos(q2 + q3 + q4), 0 ;
 0 , -sin(q1), -sin(q1),  -sin(q1), 0.5*cos(q2 - 1.0*q1 + q3 + q4) + 0.5*cos(q1 + q2 + q3 + q4);
 0 , cos(q1) , cos(q1) , cos(q1) , 0.5*sin(q1 + q2 + q3 + q4) - 0.5*sin(q2 - 1.0*q1 + q3 + q4) ;
 1, 0 , 0 , 0 ,- 1.0*sin(q2 + q3 + q4)];
Jacobian_sim = round(Jacobian_sim);

% performing rank test 
jrank = rank(Jacobian_sim);
if( jrank == 5)
    jmatrix= Jacobian_sim; 
else 
     jmatrix= Jacobian_sim; 
     disp(" this will be a singularity position")
     % The assigning  of the jamtrix is done in order to output the jacobian
     % , even if in a singualr position. We havent solved for singular
     % positions but we get an approximate jacobian for that point.
end 
jmatrix;
qdot= [qdot(1) ; qdot(2) ;qdot(3) ;qdot(4) ;qdot(5)];
e_vel= jmatrix * qdot;



end