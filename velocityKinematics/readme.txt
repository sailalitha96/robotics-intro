The scripts 

FK_Velocity takes in q and qdot and returns the e_Vel 

IK_velocity takes in e_vel and q and returns qdot.

1. For obtaining circle

Execute circulartraj file. Specify the radius and run the script . The plot would appear at once after collecting all the points

2. For obtaining a straight line 

Execute the straightline file. Provide start and end positions. Change velocities as needed. Run the script to get the straight line. 

3. For obtaining a spiral 

Execute the spiral file . Modify the R and a and exceute the file to get the desired spiral trajectory . 

4. Error plotting 

execute the error plotting file. For a given radius , modify to get different graphs. 

5. Tilted circle , 

The tilt is fixed at pi/ 4 , can change the start position of the lynx and radius of the circle drawn. Execute the file to obtain the desired trajectory. 

6. File -Independent joints 

The independent joints file can be modifed to check which trajectory a given joint takes for constant qdot. 

7. File - constantqdot 

This file provides a plot of the closed loop structure that the end effector takes when all the joint velocoties are constant. 