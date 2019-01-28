%% we used this script to obtain the jacobian matrix

syms q1 q2 q3 q4 q5 d1 a2 a3 d5 lg 
% q1= 90;
% q2= 0 ;
% q3= 0 ;
% q4= 90 ;
% q5=0 ;

T1 = sym(zeros(4,4));
T2 = sym (zeros(4,4));
T3 = sym (zeros(4,4));
T4 = sym (zeros(4,4));
T5 = sym (zeros(4,4));
T01 = sym(zeros(4,4));
T02 = sym(zeros(4,4));
T03 = sym(zeros(4,4));
T04 = sym(zeros(4,4));
T05 = sym(zeros(4,4));
X= sym (zeros(6,4));
d1 = 76.2; % base height (table to center of joint 2)
a2 = 146.05; % shoulder to elbow length
a3 = 187.325; %elbow to wrist length
d5 = 76.2; %wrist to base of gripper
lg = 28.575; %length of gripper


T1 =  [cos(q1) -sin(q1)*cos(-pi/2)     sin(q1)*sin(-pi/2)  0;
        sin(q1)  cos(q1)*cos(-pi/2)     -cos(q1)*sin(-pi/2)  0;
              0            sin(-pi/2)            cos(-pi/2) d1;
              0                     0                  0     1];
T2 = [cos(q2-(pi/2)) -sin(q2-(pi/2))  0         a2*cos(q2-(pi/2));
      sin(q2-(pi/2))  cos(q2-(pi/2))  0           a2*sin(q2-(pi/2));
              0                   0  1                     0;
              0                   0  0                     1];
T3 = [cos(q3+(pi/2)) -sin(q3+(pi/2))  0   a3*cos(q3+(pi/2));
      sin(q3+(pi/2))  cos(q3+(pi/2))  0   a3*sin(q3+(pi/2));
              0                        0  1                     0;
              0                        0  0                     1];
T4 = [cos(q4-(pi/2)) -sin(q4-(pi/2))*cos(-pi/2)   sin(q4-(pi/2))*sin(-pi/2)   0;
      sin(q4-(pi/2))  cos(q4-(pi/2))*cos(-pi/2)  -cos(q4-(pi/2))*sin(-pi/2)   0;
              0                          sin(-pi/2)                    cos(-pi/2)   0;
              0                                   0                             0   1];

T5 = [cos(q5) -sin(q5)      0        0;
      sin(q5)  cos(q5)      0        0;
              0          0  1       d5;
              0          0  0        1];
          
 T6 = [ 1  0  0   0;
       0  1  0   0;
       0  0  1  lg;
       0  0  0   1];

X(1,:) = [0 0 0 1];

%Position of Second Joint (Shoulder Revolute)
X(2,:) = (T1*[0;0;0;1])';

%Position of Third Joint (Elbow Revolute)
X(3,:) = (T1*T2*[0;0;0;1])';

%Position of Fourth Joint (1st Wrist)
X(4,:) = (T1*T2*T3*[0;0;0;1])';

%Position of Fifth Joint (2nd Wrist)
X(5,:) = (T1*T2*T3*T4*[0;0;34;1])';

%Position of Gripper (Base of the Gripper)
X(6,:) = (T1*T2*T3*T4*T5*[0;0;0;1])';

%Outputs the 6x3 of the locations of each joint in the Base Frame
jointPositions = X(:,1:3);

z0 = [ 0; 0; 1] ;
z1 = T1(1:3,3);
T02 = T1*T2;
z2= T02(1:3,3);
T03 = T1*T2*T3 ;
z3 = T03( 1:3,3);
T04 = T1*T2*T3*T4 ; 
z4 = T04(1:3,3);
T05 = T1*T2*T3*T4*T5;
z5 = T05(1:3,3);

z_rows =  [z0,z1,z2,z3,z4 ,z5 ]; % ;z2 ;z3; z4 ;z5 ]
diste0 =X(6, 1:3)' - X(1,1:3)';
diste1= X(6, 1:3)' - X(2,1:3)';
diste2= X(6, 1:3)' - X(3,1:3)';
diste3= X(6, 1:3)' - X(4,1:3)';
diste4= X(6, 1:3)' - X(5,1:3)';
Jv1 = cross(z0,diste0);
Jv2 = cross(z1,diste1);
Jv3 = cross(z2, diste2);
Jv4 = cross(z3, diste3);
Jv5 = cross(z4, diste4); 
Jw1= z0 ;
Jw2= z1 ;
Jw3 = z2;
Jw4 = z3;
Jw5= z4;
Jv = [ Jv1 , Jv2 , Jv3 , Jv4 , Jv5];
Jw = [ Jw1 , Jw2 , Jw3 , Jw4 ,Jw5];
J = vertcat(Jv , Jw);
jacob= simplify(J);
jacobian = vpa(jacob);

