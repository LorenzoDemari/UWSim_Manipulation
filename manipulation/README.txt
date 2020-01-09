Luca Cartosio, Lorenzo De Mari

----Cooperative Manipulation for a common task-------

README

Repository contents:
- simulation_photos folder
- images folder 
- Complete_video.org
- manipulation package
- inertial.png

The folders contain images of the experiment or the software architecture.

The Complete_video.org shows a record of the complete experiment.

In the manipulation package there is the code developed, in particular the nodes:
-InverseKinematicProblem.cpp
-goalDetection.cpp
-gotoBox.cpp

********************************************************************************************
HOW TO SET THE ENVIRONMENT

In the operating system Ubuntu 16.4:
-Install ROS kinetic following the instructions at the link http://wiki.ros.org/kinetic/Installation
-Then install UWSim simulator following the instractions at the link http://www.irs.uji.es/uwsim/wiki/index.php?title=Installing_UWSim

-----------------------------

2. in the file /uwsim/data/scenes/g500ARM5.urdf

add the following lines to <link name="part4_jaw1"> and <link name="part4_jaw2"> as shown in inertial.png

<inertial>
   <mass value="10"/>
   <inertia ixx="100" ixy="0" ixz="0" iyy="100" iyz="0" izz="100"/>
</inertial>

-------------------------------

3. copy the file myscene.xml contained in manipulation/data/scenes in the folder uwsim/data/scenes

-------------------------------

4.  copy the file pegaprile.obj contained in manipulation/data/objects in the folder /home/$user$/.uwsim/data/objects

********************************************************************************************

HOW TO RUN THE CODE

Open 4 terminals:

1. in the first terminal run the roscore 
roscore

2. in the second terminal open the simulator
rosrun uwsim uwsim --configfile /home/lorenzo/UWSim/src/underwater_simulation/uwsim/data/scenes/myscene.xml

3. in the third terminal run the launchfile motion.launch provided in the folder /manipulation/launch
roslaunch manipulation motion.launch

once the 2 gripper are closed:

3. stop the execution in the third terminal

4. in the fourth terminal run the launchfile task.launch provided in the folder /manipulation/launch
roslaunch manipulation task.launch