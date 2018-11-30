%%%% PROCEEDING TO CALCULATE IK %%%%%%%%

function [q is_possible] = IK_lynx _pennkey(T0e)
% Input:    T - 4 x 4 homogeneous transformation matrix, representing 
%               the end effector frame expressed in the base (0) frame
%               (position in mm)

% Outputs:  q - a 1 x 5 vector of joint inputs [q1,q2,q3,q4,q5] (rad) which
%               are required for the Lynx robot to reach the given 
%               transformation matrix T
% 
%           is_possible - a boolean set to true if the provided
%               transformation T is achievable by the Lynx robot, ignoring
%               joint limits
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
is_possible = true;

T = T0e;

d5 =76.2;
lg=28.575;
a2 = 146.05;
a3 = 187.325;
d1 = 76.2;
x= T(1,4);
y= T(2,4);
z= T(3,4); 

%%%%% CHECKING IF POSSIBLE %%%%%%%%

%% CHECK 1
radius = d5+lg+a3+a2;
if (sqrt(x^2+y^2+(z-d1)^2))>radius
    is_possible = false;
end

%% CHECK 2
Cond1 = T (1,1)^2 + T (2,1)^2 + T (3,1)^2;
Cond2 = T (1,2)^2 + T (2,2)^2 + T (3,2)^2;
Cond3 = T (1,3)^2 + T (2,3)^2 + T (3,3)^2;

if ((abs(Cond1-1)>0.1)||(abs(Cond3-1)>0.1)||(abs(Cond2-1)>0.1))
    is_possible = false;
end

%% CHECK3

N = [T(1,1), T(2,1), T(3,1)];
S = [T(1,2), T(2,2), T(3,2)];
A = [T(1,3), T(2,3), T(3,3)];

AP = cross(N,S);

if abs(AP-A)>0.1
    is_possible = false;
end

% Defining joint limits:

lowerLim = [-pi/2 -pi/2 -1.8 -1.9 -2];
upperLim = [pi/2 pi/2 1.7 1.7 pi/2];

p = T(1:3,4) - (d5+lg)*T(1:3,3); % position of center of wrist

q1= atan2(p(2),p(1)); % theta 1 is computed

val1 = (a2^2 + a3^2 - (p(3) - d1)^2 -(p (1)^2 + p (2)^2))/(2*a2*a3);
% q3 = pi/2 - acos(val1) % thetha 3 is computed
q3 = asin(val1);

% The below matrix represents all possible solutions of Q. We will be
% eliminating them as the code progresses.

qp = [q1 q1 q1+pi q1+pi ; q3 pi-q3 q3 pi-q3 ; 0 0 0 0; 0 0 0 0 ; 0 0 0 0];
q = zeros(1,5);
count = 0; % initializing count for checking if a solution is possible

for c=1:4
        a = qp(1,c);
        if (p(1)<0 && (qp(1,c)< lowerLim(1) ||  (qp(1,c)> upperLim(1))))
            qp(1,c) = atan2(abs(p(2)),abs(p(1)));
            if (p(2)>0)
                qp(1,c) = -qp(1,c);
            elseif (p(2)<0)
                qp(1,c) = qp(1,c);
            end
        end
        if (qp(1,c)< lowerLim(1) ||  (qp(1,c)> upperLim(1)) || isreal(qp(1,c))==0) % if it is outside the joint limits , should set flag true
            
            count = count + 1;
            continue; % breaks the cycle and increments to next column
        end
        if (qp(2,c)< lowerLim(3) || (qp(2,c)> upperLim(3)) || isreal(qp(2,c))==0)
            count = count + 1;
            continue; % breaks the cycle and increments to next column
        end
        
        
       if c==1
            b = pi/2 - atan2(p(3) - d1 , sqrt(p (1)^2 + p (2)^2)) + atan2(-a3*cos(qp(2,c)),a2-a3*sin(qp(2,c)));
       elseif c==2
            b = pi/2 - atan2(p(3) - d1 , sqrt(p (1)^2 + p (2)^2)) + atan2(-a3*cos(qp(2,c)),a2-a3*sin(qp(2,c)));
       elseif c==3
           b = pi/2 - atan2(p(3) - d1 , -sqrt(p (1)^2 + p (2)^2)) + atan2(-a3*cos(qp(2,c)),a2-a3*sin(qp(2,c)));
       elseif c==4
           b = pi/2 - atan2(p(3) - d1 , -sqrt(p (1)^2 + p (2)^2)) + atan2(-a3*cos(qp(2,c)),a2-a3*sin(qp(2,c)));
       end

        qp(3,c) = b;
     
       
        if (qp(3,c)< lowerLim(2) ||  (qp(3,c)> upperLim(2)) || isreal(qp(3,c))==0)
            count = count + 1;
            continue; % breaks the cycle and increments to next column
        end
        % you need to compute o new for each value of p, same thing you did for q2
        T3 = [cos(qp(2,c)+(pi/2)) -sin(qp(2,c)+(pi/2))  0   a3*cos(qp(2,c)+(pi/2));sin(qp(2,c)+(pi/2))  cos(qp(2,c)+(pi/2))  0   a3*sin(qp(2,c)+(pi/2));0  0  1 0;0   0  0  1];
        T2 = [cos(qp(3,c)-(pi/2)) -sin(qp(3,c)-(pi/2))  0   a2*cos(qp(3,c)-(pi/2)); sin(qp(3,c)-(pi/2))  cos(qp(3,c)-(pi/2))  0   a2*sin(qp(3,c)-(pi/2)); 0  0  1   0; 0  0  0 1];  
        T1 = [cos(a) -sin(a)*cos(-pi/2)  sin(a)*sin(-pi/2)  0; sin(a)  cos(a)*cos(-pi/2) -cos(a)*sin(-pi/2)  0; 0 sin(-pi/2) cos(-pi/2) d1;0 0 0 1];
        
        T03 = T1*T2*T3;
        T03 = T03';
        o=T03*T;
        
        qp(4,c)= atan2(o(2,3),o(1,3));
        if (qp(4,c)< lowerLim(4) ||  (qp(4,c)> upperLim(4)) || isreal(qp(4,c))==0)
            count = count + 1;
            continue; % breaks the cycle and increments to next column
        end
        % you need to compute o new for each value of p, same thing you did for q2
        qp(5,c) = atan2(-o(3,1), -o(3,2));
        if (qp(5,c)< lowerLim(5) ||  (qp(5,c)> upperLim(5)) || isreal(qp(5,c))==0)
            count = count + 1;
            continue; % breaks the cycle and increments to next column
        end
        
        % assigning values to 'q'
        q(1,1) = qp(1,c);
        q(1,2) = qp(3,c);
        q(1,3) = qp(2,c);
        q(1,4) = qp(4,c);
        q(1,5) = qp(5,c);
end

if count==4
    disp('Cannot be executed, all joint limits are violated (or) returning complex numbers, returning reference angle');
    q = zeros(1,5);
end
    
