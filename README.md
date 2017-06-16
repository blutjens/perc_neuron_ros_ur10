# Real-time teleoperation of industrial robots with the motion capture system Perception Neuron
Bachelor's thesis in Engineering Science

Bjorn Lutjens (bjoern.luetjens@tum.de), Dr. Emmanuel Carlos Dean-Leon

Institute for Cognitive System, Technical University of Munich

## Transmits motion data from Perception Neuron (PN) suit onto UR10

Sends PN motion data from Windows over rosserial to ROS. 
ROS node receives data and publishes on tf. tf skeleton is transformed by
inverse kinematics into UR10 joint states. Joints are checked on safety 
regulations and published to UR10.

For explanation, setup instruction and detailed exeuction instructions 
read thesis: perc_neuron_lutjens_ics_thesis.pdf

View follow_hand_demo_290716.ogv for a demonstration of the starting process.

-------
# On ROS Machine

## Terminal 1 
 - $ rosclean purge -y 
 - $ roscore

## For every new terminal:
 Direct to workspace 
  - $ cd ~/ros/workspaces/personal_ws
 Source bash file 
  - $ source personal_ws/devel/setup.bash 

## Terminal 2
 Start the rosserial server on Linux, such that the module on Windows can transfer Perception Neuron data
  - $ rosrun rosserial_server socket_node
-------
# On Windows Machine
## Start Perception Neuron GUI ("Axis Neuron x64.exe")
 Play movement data:
  - Put on and connect the suit
  - Stand as far away from magnetic sources as possible
  - Calibrate the suit by following the software´s instructions in Perc_Neuron_manuals/AxisUserGuide.pdf 
  - Alternatively, test with recorded movement data: open and run sample recorded movements …/samplePNmove.raw or Noitom/Axis Neuron/Motion Files/…

## Start broadcast to ROS:
 - Open cmd window as administrator
 - Navigate to pn_ros_windows/…/windows/PerceptionNeuronROSserial.exe
 - Open windows/config.txt and adapt IP addresses for ROS Serial Server and Axis Neuron (found in Axis Neuron settings) 
 - $ PerceptionNeuronROSserial.exe

## Assure real-time behavior of PerceptionNeuronROSserial.exe
 Assign Real-Time task priority to PerceptionNeuronROSserial in Windows task-manager
 - Open task-manager -> Details -> PerceptionNeuronROSserial.exe or shell -> Priorität festlegen -> Echtzeit
 If possible, assign one CPU core to solely manage PerceptionNeuronROSserial.exe 
## Known Bugs, caused by Axis Neuron Software Beta Version:    
 - Problem 1: PerceptionNeuronROSserial.exe shuts down due to registration fail in Perception Neuron server.
 - Solution 1: Restart until it works. 3-4 times.
 - Problem 2: Axis Neuron does not publish data
 - Solution 2: Open Axis Neuron->File -> Settings and close them again, because Axis Neuron wouldn’t broadcast data without having opened the settings.
-------
# Continue on ROS Machine:

## Terminal 3
 Start to publish the data from the Rosserial_server to tf
  - $ rosrun perc_neuron_tf_broadcaster perc_neuron_tf_broadcaster_node

## Terminal 4
 Start the UR10 simulation and rviz. simulation is listening to /ur10_arm_joint_states
  - $ roslaunch tum_ics_ur10_bringup bringUR10.launch

## Terminal 5
 Only for use with a REAL robot: build a connection between the robot and the ROS machine
  - $ roslaunch tum_ics_ur_robot_manager  robot_script_manager_ur10.launch

## Terminal 6
 For simulation:
  - $ roslaunch tum_ics_ur_robot_controllers testJointCtrl_ur10.launch
 Real Robot (WARNING the robot is now active) HAND on the red button, EYES on the robot!!!
  - $ roslaunch tum_ics_ur_robot_controllers testJointCtrl_ur10.launch ROBOT:=real
 
 Follow steps in thesis 13.1.2.2 if tum_ics_ur_robot_controllers isn’t existing or start follow_hand in simulation mode

## Terminal 7
 ### Start the main program to transfer the tf data from the Perception Neuron onto the robot:
  For simulation mode:
   - ONLY without real robot!
   - Bypasses an additional safety controller and does not retrieve real robot initial position. 
   - Set /publishToController in ur10_perc.launch to false
   - Publishes on /ur10_arm_joint_states
  
  For real robot mode:
   - Set /publishToController in ur10_perc.launch to true
   - Publishes on /joint_desired_cmd
   - Control safety restriction parameters in ur10_perc.launch
  
  $ roslaunch follow_hand ur10_perc.launch

