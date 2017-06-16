/* perception_neuron_tf_broadcaster.cpp
 *
 * Copyright (C) 2015 Alexander Rietzler, Simon Haller
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD license.  See the LICENSE file for details.
 */

/* Changes by Bjorn Lutjens 15th June 2017 (bjoern.luetjens@tum.de)
 * - updated to 60Hz rate
 * - receive one motion frame in one message
 * - timely ordered skeleton model
 * - message frame ID label
 * - single-threaded spinner
 * - tf skeleton model update
 * - commented code
*/

#include <ros/ros.h>
#include <ros/package.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Dense>

#include <chrono> // High precision event timer
#include <std_msgs/Float32MultiArray.h> // Output msg
#include <ros/callback_queue.h>

class NeuronBroadcaster {
 public:
  NeuronBroadcaster(ros::NodeHandle& nh) : nh_(nh) {
    // Incoming child data sequence is determined by perception_neuron
    // DataReader API. Link_children_name is the ID of the coordinate
    // frame this transform defines:
    link_children_names_=std::vector<std::string>{
        "Hips","RightUpLeg","RightLeg","RightFoot","LeftUpLeg","LeftLeg",
        "LeftFoot","Spine","Spine1","Spine2","Spine3","Neck","Head",
        "RightShoulder","RightArm","RightForeArm","RightHand",
        "RightHandThumb1","RightHandThumb2","RightHandThumb3",
        "RightInHandIndex","RightHandIndex1","RightHandIndex2",
        "RightHandIndex3","RightInHandMiddle","RightHandMiddle1",
        "RightHandMiddle2","RightHandMiddle3","RightInHandRing",
        "RightHandRing1","RightHandRing2","RightHandRing3","RightInHandPinky",
        "RightHandPinky1","RightHandPinky2","RightHandPinky3","LeftShoulder",
        "LeftArm","LeftForeArm","LeftHand","LeftHandThumb1","LeftHandThumb2",
        "LeftHandThumb3","LeftInHandIndex","LeftHandIndex1","LeftHandIndex2",
        "LeftHandIndex3","LeftInHandMiddle","LeftHandMiddle1","LeftHandMiddle2",
        "LeftHandMiddle3","LeftInHandRing","LeftHandRing1","LeftHandRing2",
        "LeftHandRing3","LeftInHandPinky","LeftHandPinky1","LeftHandPinky2",
        "LeftHandPinky3"
    };

    // Link_parent_name is the ID of the coordinate frame in which
    // the transform is defined:
    link_parents_names_=std::vector<std::string>{
        "WorldPerceptionNeuron","Hips","RightUpLeg","RightLeg","Hips",
        "LeftUpLeg","LeftLeg","Hips","Spine","Spine1","Spine2","Spine3","Neck",
        "Spine3","RightShoulder","RightArm","RightForeArm","RightHand",
        "RightHandThumb1","RightHandThumb2","RightHand","RightInHandIndex",
        "RightHandIndex1","RightHandIndex2","RightHand","RightInHandMiddle",
        "RightHandMiddle1","RightHandMiddle2","RightHand","RightInHandRing",
        "RightHandRing1","RightHandRing2","RightHand","RightInHandPinky",
        "RightHandPinky1","RightHandPinky2","Spine3","LeftShoulder","LeftArm",
        "LeftForeArm","LeftHand","LeftHandThumb1","LeftHandThumb2",
        "LeftHand","LeftInHandIndex","LeftHandIndex1","LeftHandIndex2",
        "LeftHand","LeftInHandMiddle","LeftHandMiddle1","LeftHandMiddle2",
        "LeftHand","LeftInHandRing","LeftHandRing1","LeftHandRing2",
        "LeftHand","LeftInHandPinky","LeftHandPinky1","LeftHandPinky2"
  };

  std::string topic_name = "/perception_neuron/data";
  subscriber_ = nh_.subscribe<std_msgs::Float32MultiArray>(
      topic_name, 5, boost::bind(&NeuronBroadcaster::callback, this, _1));
 }

 void sendStaticTransform() {
   // Static root transform is used to rotate whole skeleton model
   tf::Transform world_frame;
   world_frame.setOrigin(tf::Vector3(0, 0, 0));
   world_frame.setRotation(tf::Quaternion(0.70711, 0, 0, 0.70711));
   tf_broadcaster_.sendTransform(tf::StampedTransform(
       world_frame, ros::Time::now(), "world", "WorldPerceptionNeuron"));
 }

 ~NeuronBroadcaster(){};

 private:
 ros::NodeHandle nh_;
 ros::Subscriber subscriber_;
 std::vector<std::string> link_children_names_, link_parents_names_;
 tf::TransformBroadcaster tf_broadcaster_;

 // Uncomment to check on lost messages:
 // Send msgID instead of frameID from Windows to check on lost msgs.
 // int lastMsgID = 0;
 // int msgID = 0;

 // Frequency measurement of callback function
 // ros::Time lastReceiveTime = ros::Time::now();
 int tfFrameID = 0;

 // Converts rotation given in degrees to quaternion data
 void eulerToQuaternion(float eulerY, float eulerX, float eulerZ,
                        tf::Quaternion& q) {
   Eigen::Matrix3f rxyz, rx, ry, rz;

   rx = Eigen::AngleAxisf(eulerX * M_PI / 180, Eigen::Vector3f::UnitX());
   ry = Eigen::AngleAxisf(eulerY * M_PI / 180, Eigen::Vector3f::UnitY());
   rz = Eigen::AngleAxisf(eulerZ * M_PI / 180, Eigen::Vector3f::UnitZ());

   // Check Ordering in Axis Neuron->Settings->Output Format! Here = YXZ
   rxyz = ry * rx * rz;

   Eigen::Quaternionf qf(rxyz);

   q.setW(qf.w());
   q.setX(qf.x());
   q.setY(qf.y());
   q.setZ(qf.z());
 }
 void callback(const std_msgs::Float32MultiArrayConstPtr& bone_data) {
   // startIdx gives reference to the start of bone_data array position regarding
   // the current limb
   uint startIdx = 0;

   // joint_index is indicating which parent belongs to which child.
   // It gives the actual transforms number in link_children_names
   // joint_index = 0 : Hips
   //...
   // Size of PercNeuron bone data array = (59(bones) * 6(3rot+3trans) = 354
   for (uint joint_index = 0; joint_index < bone_data->data.size() / 6;
        joint_index++) {
     startIdx = joint_index * 6;

     tf::Transform pose;
     float eulerY, eulerX, eulerZ;
     tf::Quaternion rotation;
     tf::Vector3 position;

     position.setX(0.01 * bone_data->data[startIdx]);  // conversion to meters
     position.setY(0.01 * bone_data->data[startIdx + 1]);
     position.setZ(0.01 * bone_data->data[startIdx + 2]);

     eulerY = bone_data->data[startIdx + 3];
     eulerX = bone_data->data[startIdx + 4];
     eulerZ = bone_data->data[startIdx + 5];

     eulerToQuaternion(eulerY, eulerX, eulerZ, rotation);

     pose.setOrigin(position);
     pose.setRotation(rotation);

     // Set Hips to equal WorldPerceptionNeuron and TF-UR10 orientation
     if (joint_index == 0) {
       pose.setOrigin(tf::Vector3(0, 0, 0));
       rotation.setRPY(0, 3 * M_PI / 2,
                       0);  // rotate around y of WorldPerceptionNeuron
       pose.setRotation(rotation);
     }

     // Size of one stamped transform is 152 byte
     tf_broadcaster_.sendTransform(tf::StampedTransform(
         pose, ros::Time::now(), link_parents_names_.at(joint_index),
         link_children_names_.at(joint_index)));
   }

   ROS_INFO_STREAM("PercNeuron frameID: " << bone_data->layout.dim[0].label
                                          << "| TF frameID: " << tfFrameID);
   tfFrameID++;

   // lastReceiveTime = ros::Time::now();
   /* Uncomment to check on lost msgs.
   std::string::size_type sz;
   msgID = std::stoi(bone_data->layout.dim[0].label, &sz);
   if( msgID - lastMsgID > 1){ // skipped a message
       ROS_INFO_STREAM("LOST A MESSAGE/n/n/n");
   }
   lastMsgID = msgID;
   */
 }
};

int main(int argc, char** argv) {
  ROS_INFO_STREAM("started perc_neuron_tf_broadcaster");

  ros::init(argc, argv, "perception_neuron_tf_broadcaster_node",
            ros::init_options::AnonymousName);

  ros::NodeHandle nh;

  NeuronBroadcaster neuronBroadcaster(nh);

  // high precision timer
  // ^^^^^^^^^^^^^^^^^^^^
  double frameRate = 60;
  double cycleTime = 1 / frameRate;  // In seconds
  std::chrono::high_resolution_clock::time_point lastPublishTime =
      std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsedTime;
  // Std deviation of ros::Duration::sleep() function in sec:
  double varianceOfStdSleep = 0.002;

  ros::CallbackQueue::CallOneResult callOneResult;
  while (nh.ok()) {
    // Returns 0 if callback pulled from queue.
    callOneResult = ros::getGlobalCallbackQueue()->callOne();

    neuronBroadcaster.sendStaticTransform();

    // High precision timer for rate establishment
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    elapsedTime = std::chrono::duration_cast<std::chrono::duration<double>>(
        std::chrono::high_resolution_clock::now() - lastPublishTime);

    // Equivalate time s.t. varianceOfStdSleep time is left over for
    // high precision timer
    if (cycleTime - elapsedTime.count() - varianceOfStdSleep > 0) {
      ros::Duration dur(cycleTime - elapsedTime.count() -
                        varianceOfStdSleep);
      dur.sleep();
    }
    // Spin until cycleTime is reached
    while (elapsedTime.count() < cycleTime) {
      elapsedTime = std::chrono::duration_cast<std::chrono::duration<double>>(
          std::chrono::high_resolution_clock::now() - lastPublishTime);
    }
    lastPublishTime = std::chrono::high_resolution_clock::now();

    if (callOneResult == 0) {
      ROS_INFO_STREAM("Callback cycle time: " << 1000 * elapsedTime.count());
    } else {
      ROS_INFO_STREAM(
          "No frame received. Control Windows PerceptionNeuronROSserial.");
    }
  }

  ROS_INFO_STREAM(
      "This is a utility to publish perception neuron bone data to tf");
  ROS_INFO_STREAM("Type any key when you are done");
  std::string mode;
  std::cin >> mode;

  ROS_INFO_STREAM("Bye!");
  return 0;
}
