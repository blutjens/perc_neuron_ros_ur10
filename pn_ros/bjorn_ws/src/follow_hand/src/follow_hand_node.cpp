/* follow_hand_node.cpp
*
* Lets the UR10 end effector follow the users right hand trajectory.

* Takes the TF data, which was produced by the Perception Neuron suit
* and sent to tf by perc_neuron_tf_broadcaster. Calculates continually
* the goal position of the UR10 via IK, checks it to be inside joint
* restrictions on angle and velocity and publishes them to the UR10.
*
* Copyright (C) 2017 Bjorn Lutjens (bjoern.luetjens@tum.de),
* Dr. Emmanuel Carlos Dean Leon
* All rights reserved.
*
* This software may be modified and distributed under the terms
* of the GNU license.  See the LICENSE file for details.
*/

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <string>

#include <math.h>
#include <stdio.h>

#include <ctime>  // runtime measurement

#include <qt4/Qt/qvector.h>

#include <tumtools/Math/MathTools.h>  // Calculation of robot trajectory
                                      // during initialization

#include <ur_kinematics/ur_kin.h>

using namespace std;


// Looks up a tf transfrom and pushes them into an array for calculating
// the goal transform
// Takes the tf from transOrigin to transDestination in tfListener and
// pushes them in transArray
// Returns 0 for success, -1 for failure
int pushIntoTransformArray(std::string transOrigin,
                           std::string transDestination,
                           std::vector<tf::Transform>* transArray,
                           tf::TransformListener* tfListener) {
  tf::StampedTransform transform;
  try {
    tfListener->lookupTransform(transOrigin, transDestination, ros::Time(0),
                                transform);
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
    return -1;
  }
  transArray->push_back(
      tf::Transform(transform.getRotation(), transform.getOrigin()));
  return 0;
}

struct robotRestrictions {
  // Maximum joint velocity in rad per sec
  double maxVel;
  bool checkElbowRestriction;
  // angle between base link and 3rd joint=
  // minimum inner angle of elbow of robot is in interval [0, M_PI]
  double innerElbowMin;
  double innerElbowMax;
};

// Interface to publish messages to robot controller
struct pubIntf {
  bool publishToController;  // True: node publishes to robot controller, False:
                             // robot publishes directly to joint states
  ros::Publisher jointPosPublisher;  // Publishes desired joint values
  sensor_msgs::JointState
      jointMsg;       // Published message with desired joint values
  double jointPubCycleTime;  // Cycle time at which joint values are published
};

// Interface to tf Data, which is published by Perception Neuron TF Broadcaster
struct tfPercNeuronInterface {
  std::vector<std::string> bodyJointsRightHand;  // Array with tf names of
                                        // Perception Neuron joints. Necessary
                                        // to listen to them with tfListener
  tf::TransformListener
      tfListener;  // tf listener to data from external tf broadcaster
  tf::Transform transBaseLinkRightHand;  // goal Transform for IK calculations,
                                         // robot base_link to RightHand
                                         // dpushIntoTransformArrayirectly
  tf::TransformBroadcaster
      tfBroadcaster;  // broadcasts goal transform to tf to visualize it in rviz

  // Get tf Transform / Vector from base_link of ur# to RightHand of PercNeuron
  // The UR# End Effector follows this transform
  // returns 0 for success, -1 for failure
  int getTransBaseLinkRightHand() {
    tf::Transform
        transHipsRightHand;  // transform from Hips to RightHand directly
    std::vector<tf::Transform> transformArray;
    int success =
        0;  // = 0 for success of getTransBaseLinkRightHand; < 0 for failure

    // Get Transform from Hips to RightHand of PercNeuron
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // fill transformArray with transforms from "Hips" to "RightHand"; i=0 would
    // equal Hips
    for (int i = 0; i < bodyJointsRightHand.size() - 1; i++) {
      success += pushIntoTransformArray(
          bodyJointsRightHand.at(i), bodyJointsRightHand.at(i + 1),
          &transformArray, &tfListener);  // returns 0 for success -1 for failure
      if (success != 0) {
        printf("\nNo tf transformation found from Hips to RightHand");
        printf(
            "\nPlease start rosserial_server node, perc_neuron_tf_broadcaster "
            "or Perception Neuron publisher on Windows.");
        return -1;
      }
    }
    // Calculate direct vector from Hips to RightHand
    transHipsRightHand = transformArray.at(0);
    for(int i = 1; i < transformArray.size(); i++){
        transHipsRightHand = transHipsRightHand.operator*=(transformArray.at(i));
    }
    transformArray.clear();
    // tfBroadcaster.sendTransform(tf::StampedTransform(
    //    transHipsRightHand, ros::Time::now(), "/Hips", "/hipsToHand"));


    // Calculate transformation matrix from ur# base_link to RightHand
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    success += pushIntoTransformArray("/base_link", "/world", &transformArray,
                                      &tfListener);
    success += pushIntoTransformArray("/world", "/WorldPerceptionNeuron",
                                      &transformArray, &tfListener);
    success += pushIntoTransformArray("/WorldPerceptionNeuron", "/Hips",
                                      &transformArray, &tfListener);
    /*  // Uncomment if tf /hipsToHand is published externally:
        success += pushIntoTransformArray("/Hips", "/hipsToHand",
            &transformArray, &tfListener);
        transBaseLinkRightHand =
            transformArray.at(0).inverseTimes(transformArray.at(1).operator*=(
                transformArray.at(2).operator*=(transformArray.at(3))));
    */
    if (success != 0) {
      printf("\nNo tf Transformation found from base_link to RightHand.");
      printf(
          "\nPlease assure that tf robot model and tf perception neuron model "
          "are being published.");
      return -1;
    }
    // Calculate transformation matrix from base link to RightHand
    transBaseLinkRightHand =
        transformArray.at(0).inverseTimes(transformArray.at(1).operator*=(
            transformArray.at(2).operator*=(transHipsRightHand)));
    transformArray.clear();

    // Rotate goal Transform M_PI around y to turn end effector into the
    // direction of the real hand/palm; not necessary for LeftHand
    tf::Transform rotationRightHandToEE(
        tf::Quaternion(tf::Vector3(0, 1, 0), M_PI),
        tf::Vector3(0, 0, 0));  // rotates the tf frame of RightHand from
                                // pointing to the body to poiting outwards
    transBaseLinkRightHand =
        transBaseLinkRightHand.operator*(rotationRightHandToEE);

	// Broadcast the transform from base_link to the RightHand
	// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    tfBroadcaster.sendTransform(
        tf::StampedTransform(transBaseLinkRightHand, ros::Time::now(),
                             "/base_link", "/baseLinkToHand"));
    return 0;
  }
};


struct ik {
  double* transForIK;  // 4x4 homogeneous transformation matrix in workspace,
                       // input for ur_kinematics IK calculations
  double* qIKSolMat;   // numSols * 6 vector, solution in joint space of IK
                       // calculations
  int numSols;         // number of analytial solutions of IK
  int ikSolIndex;         // index of selected solution (elbow up, down...)
  std::vector<double> qIKSolVec;  // Selected solution to get published on robot

  int calcqIKSolVec(const tfPercNeuronInterface& tfPNIntf) {
    // Form goal tf transform into required input double* for ur_kinematics
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // rotational part of homogeneous transformation matrix
    for (int i = 0; i <= 2; i++) {
      transForIK[i * 4 + 0] =
          tfPNIntf.transBaseLinkRightHand.getBasis().getRow(i).getX();
      transForIK[i * 4 + 1] =
          tfPNIntf.transBaseLinkRightHand.getBasis().getRow(i).getY();
      transForIK[i * 4 + 2] =
          tfPNIntf.transBaseLinkRightHand.getBasis().getRow(i).getZ();
    }
    // translational part
    transForIK[3] = tfPNIntf.transBaseLinkRightHand.getOrigin().getX();
    transForIK[7] = tfPNIntf.transBaseLinkRightHand.getOrigin().getY();
    transForIK[11] = tfPNIntf.transBaseLinkRightHand.getOrigin().getZ();

    transForIK[12] = 0;
    transForIK[13] = 0;
    transForIK[14] = 0;
    transForIK[15] = 1;

    // Calculate Inverse Kinematic Solution
    // Input: 4x4 homogeneous transformation matrix from base to right hand
    // Output: 6*numSols matrix of possible joint positions to fit the end
    //      effector to the right hand. Only the joint position vector at
    //      ik.ikSolIndex will be used. No solution found, if goal transform
    //      is out of reach for ur#, case is handled by vel.ctrl.
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    numSols = ur_kinematics::inverse(transForIK, qIKSolMat);

    // Turn values of selected solution over M_PI into negative values
    // (by substraction of 2*M_PI) to use joint limited robot
    // => -M_PI < publishedJointPosition <= M_PI
    for (int j = 0; j < 6; j++) {
      if (M_PI < qIKSolMat[ikSolIndex * 6 + j])
        qIKSolMat[ikSolIndex * 6 + j] =
            qIKSolMat[ikSolIndex * 6 + j] - 2 * M_PI;
    }
    // Copy selected solution into solution vector
    qIKSolVec = {qIKSolMat[ikSolIndex * 6 + 0], qIKSolMat[ikSolIndex * 6 + 1],
                 qIKSolMat[ikSolIndex * 6 + 2], qIKSolMat[ikSolIndex * 6 + 3],
                 qIKSolMat[ikSolIndex * 6 + 4], qIKSolMat[ikSolIndex * 6 + 5]};
    return 0;
  }
};

void printVector(std::vector<double>& vector, bool debugStream) {
  if (debugStream) {
    ROS_DEBUG("\n");
    for (auto& currentElement : vector) {
      ROS_DEBUG("%f ", currentElement * 180 / M_PI);
    }
  } else {
    printf("\n");
    for (auto& currentElement : vector) {
      printf("%f ", currentElement * 180 / M_PI);
    }
  }
}

// Checks the angle restrictions set on the robot joints
// returns 1, if desired angle restricts the robot restrictions
// returns 0, if desired angle is valid
int violatesJointRestriction(const struct robotRestrictions& robRestr,
                             double innerAngle) {
  if (robRestr.innerElbowMin <= innerAngle && innerAngle <= robRestr.innerElbowMax)
    return 0;
  else
    return 1;
}

// Interpolates last published position with desired goal, checks physical
// constraints on robot and publishes new desired joint values to
// simulation or controller
// Returns 0 if goalPosition got published, -1 if not
int checkAndPublishDesJointValues(ros::NodeHandle& node,
                                  std::vector<double>& goalPosition,
                                  std::vector<double>& lastPubPosition,
                                  pubIntf& pubIntf, double cycleTime,
                                  const robotRestrictions& robRestr) {

  // Initialize Variables
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^
  bool goalPossible = false;
  // -2*M_PI < deltaPosition < 2*M_PI
  std::vector<double> deltaPosition = {2 * M_PI, 2 * M_PI, 2 * M_PI,
                                       2 * M_PI, 2 * M_PI, 2 * M_PI};
  std::vector<double> publishPosition = {0, - M_PI / 2, 0,
                                         - M_PI / 2, 0, 0};

  int numberPublishes = floor(
      cycleTime /
      pubIntf.jointPubCycleTime);  // round down to not publish faster than 125 Hz
  if (numberPublishes == 0){ // publish at least once
      numberPublishes = 1;
  }
  double correctedDeltaPos = 0*M_PI;  // corrects the joint distance for the
  // limited joint robot at position jump from -M_PI to M_PI or vice versa by
  // respectively subtracting or adding 2*M_PI to the deltaPosition.

  int elbowJointIndex =
      2;  // Index of elbow joint in goal/solution/delta/lastPubPosition vector
          // position i: 0:qShoulderPan, 1:qShoulderLift, 2:qElbow, 3:qWrist1,
          // 4:qWrist2, 5:qWrist3;

  // Check reachability of goals with elbow joint restriction:
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // default possible default range: 20° - 120° = PI/9 - 2/3*PI
  if (robRestr.checkElbowRestriction == 1) {
    double innerAngleElbow = M_PI - abs(goalPosition.at(elbowJointIndex));
    // Works safe for solution 5: elbow up; shoulder right; wrist down / at body
    if (1 == violatesJointRestriction(robRestr, innerAngleElbow)) {
      goalPossible = false;
      printf(
          "\n Desired elbow angle: %f° is outside of elbow joint inner angle "
          "boundaries: %f° - %f°, wait for User to come back to robot.",
          innerAngleElbow / M_PI * 180,
          robRestr.innerElbowMin / M_PI * 180,
          robRestr.innerElbowMax / M_PI * 180);
      return -1;
    } else {
      goalPossible = true;
    }
  }

  // Differenz goalPosition / lastPubPosition bilden
  for (int i = 0; i < goalPosition.size(); i++) {
    deltaPosition.at(i) = goalPosition.at(i) - lastPubPosition.at(i);
  }

  // Check for max velocity regulation
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  for (int i = 0; i < deltaPosition.size(); i++) {
    if (deltaPosition.at(i) < -M_PI) {
      correctedDeltaPos = deltaPosition.at(i) + 2 * M_PI;
      if (correctedDeltaPos / cycleTime < robRestr.maxVel)  // maxVel excluded
        goalPossible = true;
      else {
        goalPossible = false;
        printf("\n1:Joint state i=%d reached the velocity limit: maxVel = %f°/s,"
          "desiredVel = %f°/s, desired delta Joint State: %f°, in time: %fs", i,
          robRestr.maxVel / M_PI * 180,
          correctedDeltaPos / M_PI * 180 / cycleTime,
          abs(deltaPosition.at(i)) / M_PI * 180,
          cycleTime);
        return -1;
      }
    } else if (M_PI < deltaPosition.at(i)) {
      correctedDeltaPos = deltaPosition.at(i) - 2 * M_PI;
      if (-robRestr.maxVel < correctedDeltaPos / cycleTime)
        goalPossible = true;
      else {
        goalPossible = false;
        printf("\n2:Joint state i=%d reached the velocity limit: maxVel = %f°/s,"
          "desiredVel = %f°/s, desired delta Joint State: %f°, in time: %fs", i,
          robRestr.maxVel / M_PI * 180,
          correctedDeltaPos / M_PI * 180 / cycleTime,
          abs(deltaPosition.at(i)) / M_PI * 180,
          cycleTime);
        return -1;
      }
    }
    else if (abs(deltaPosition.at(i)) / cycleTime <
             robRestr.maxVel)
      goalPossible = true;
    else {
      goalPossible = false;
      printf("\n3:Joint state i=%d reached the velocity limit: maxVel = %f°/s,"
        "desiredVel = %f°/s, desired delta Joint State: %f°, in time: %fs", i,
        robRestr.maxVel / M_PI * 180,
        deltaPosition.at(i) / M_PI * 180 / cycleTime,
        abs(deltaPosition.at(i)) / M_PI * 180,
        cycleTime);
      return -1;
    }
  }


  // Interpolate and publish solutions
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // all solutions from here on have to be possible solutions and not
  // damage the robot
  // interpolate to smoothen robot behavior
  ros::Rate rate(1 / pubIntf.jointPubCycleTime);

  for (int step = 1; step <= numberPublishes;
       step++) {  // = 2 at 60Hz main loop
    // Interpolate und set publishPosition
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    for (int i = 0; i < deltaPosition.size(); i++) {
      // Check for jump from positive to negative value or vize versa, caused
      // by negating values after ik calculation
      // So keeps boundaries -M_PI < publishPosition <= M_PI
      if (M_PI < deltaPosition.at(i)) {
        correctedDeltaPos = deltaPosition.at(i) - 2 * M_PI;
        publishPosition.at(i) =
            lastPubPosition.at(i) +
            ((float)step / (float)numberPublishes) * correctedDeltaPos;
        if (publishPosition.at(i) < -M_PI) {
          publishPosition.at(i) += 2 * M_PI;
        }
        printf(
            "\njump from negative to positive, publishPosition: %f, lastPos "
            "%f, deltaPos %f",
            publishPosition.at(i), lastPubPosition.at(i), deltaPosition.at(i));
        ROS_DEBUG_STREAM(
            "Detected M_PI < (jump from negative to positive desired joint "
            "angle)");
      } else if (deltaPosition.at(i) < -M_PI) {
        correctedDeltaPos = deltaPosition.at(i) + 2 * M_PI;
        publishPosition.at(i) =
            lastPubPosition.at(i) +
            ((float)step / (float)numberPublishes) * correctedDeltaPos;
        if (M_PI < publishPosition.at(i)) {
          publishPosition.at(i) -= 2 * M_PI;
        }
        ROS_DEBUG_STREAM(
            "Detected (jump from positive to negative desired joint angle) < "
            "-M_PI");
        printf(
            "\njump from positive to negative, publishPosition: %f, lastPos "
            "%f, deltaPos %f",
            publishPosition.at(i), lastPubPosition.at(i), deltaPosition.at(i));
      } else
        publishPosition.at(i) =
            lastPubPosition.at(i) +
            ((float)step / (float)numberPublishes) * deltaPosition.at(i);
    }
    // Send / Publish Data to Robot Controller
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    pubIntf.jointMsg.header.stamp = ros::Time::now();
    pubIntf.jointMsg.position = {
        publishPosition.at(0), publishPosition.at(1), publishPosition.at(2),
        publishPosition.at(3), publishPosition.at(4), publishPosition.at(5)};

    ROS_INFO_STREAM("Published Position: " <<
                    publishPosition.at(0) << " " << publishPosition.at(1) << " " <<
                    publishPosition.at(2) << " " << publishPosition.at(3) << " " <<
                    publishPosition.at(4) << " " << publishPosition.at(5));
    pubIntf.jointPosPublisher.publish(pubIntf.jointMsg);
    rate.sleep();
  }
  // There is no position feedback from robot ->
  // Copy last published position to serve as new current position
  std::copy_n(publishPosition.begin(), publishPosition.size(),
              lastPubPosition.begin());
  // ----------------------------

  return 0;
}

void printTransformMatrix(tf::Transform& tfTransform) {
  int i = 0;
  printf("\n%1.6f %1.6f %1.6f %1.6f\n",
         tfTransform.getBasis().getColumn(i).getX(),
         tfTransform.getBasis().getColumn(i).getY(),
         tfTransform.getBasis().getColumn(i).getZ(),
         tfTransform.getOrigin().getX());
  i = 1;
  printf("%1.6f %1.6f %1.6f %1.6f\n",
         tfTransform.getBasis().getColumn(i).getX(),
         tfTransform.getBasis().getColumn(i).getY(),
         tfTransform.getBasis().getColumn(i).getZ(),
         tfTransform.getOrigin().getY());
  i = 2;
  printf("%1.6f %1.6f %1.6f %1.6f\n",
         tfTransform.getBasis().getColumn(i).getX(),
         tfTransform.getBasis().getColumn(i).getY(),
         tfTransform.getBasis().getColumn(i).getZ(),
         tfTransform.getOrigin().getZ());
  printf("%1.6f %1.6f %1.6f %1.6f\n", 0.0, 0.0, 0.0, 1.0);
}

// Drives the robot from its default position to the initial hand position.
// Works over large distance; saves hand position s.t. user can move hand
// during intialization
// Called in the beginning and whenever the robot loses the track of the hand.
// returns -1 if not successful, 0 for success
int initializeRobotToHand(ros::NodeHandle& node, pubIntf& pubIntf,
                          tfPercNeuronInterface& tfPNIntf, ik& ik,
                          std::vector<double>& lastPubPosition,
                          std::vector<double>& defaultRobotPosition,
                          const robotRestrictions& robRestr) {
  printf(
      "\nPress Enter to drive Robot from Robot Default Position to your "
      "hand.\nPlease hold hand steady during the process.");
  getchar();
  // Get tf transform from base_link of ur# to RightHand of PercNeuron
  int success = -1;
  while (success == -1 && node.ok()) {
    success = tfPNIntf.getTransBaseLinkRightHand();  // returns 0 for success 
  }

  // Get Goal Joint Positions by Inverse Kinematics
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  ik.calcqIKSolVec(tfPNIntf);  // sets ik.qIKSolVec

  // Calculate necessary time for reinitialization process
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  double deltaAngle = 0;// difference from current robot joint angle to goal angle
  double maxDeltaAngle = 0;
  double avgRadPSec =
      robRestr.maxVel /
      10;  // by setting the avgVel to a 10th of the maxVel, robot won't
  // violate velocity restrictions at its fastest point. As exact function
  // of trajectory calculation isn't known, it will be checked again stepwise.

  // Set maxDeltaAngle
  for (int i = 0; i < lastPubPosition.size(); i++) {
    deltaAngle = abs(ik.qIKSolVec.at(i) - lastPubPosition.at(i));
    if (maxDeltaAngle < deltaAngle) {
      maxDeltaAngle = deltaAngle;
    }
  }
  double minInitDuration = maxDeltaAngle / avgRadPSec;
  printf("\nDuration of (re-)initialization: %f", minInitDuration);

  // Check for Elbow Joint Angle Restriction before driving the robot to the
  // hand position, s.t. user could start to reinitialize from new position
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  int elbowJointIndex = 2;
  double innerAngleElbow = M_PI - abs(ik.qIKSolVec.at(elbowJointIndex));
  if (1 == violatesJointRestriction(robRestr, innerAngleElbow)) {
    printf(
        "\nElbow Joint Restriction Violation. Desired inner angle should lie"
        " in between %f° and %f°, but is: %f°",
        robRestr.innerElbowMin / M_PI * 180,
        robRestr.innerElbowMax / M_PI * 180,
        innerAngleElbow / M_PI * 180);
    return -1; // abort (re-)initialization
  }

  // Set the current robot position and its goal position
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // Intialize with default position, s.t. an error in the program would
  // publish the current robot position
  lastPubPosition = defaultRobotPosition;
  std::vector<double> currentGoal = defaultRobotPosition;
  std::vector<double>& currentGoalRef = currentGoal;

  // Copy the start and goal position matrices into input for trajectory calculation
  Eigen::Matrix<double, 6, 1> defaultPosInput;  // Matrix< _Scalar, _Rows, _Cols >
  defaultPosInput << defaultRobotPosition.at(0), defaultRobotPosition.at(1),
      defaultRobotPosition.at(2), defaultRobotPosition.at(3),
      defaultRobotPosition.at(4), defaultRobotPosition.at(5);

  Eigen::Matrix<double, 6, 1> goalPosInput;
  goalPosInput << ik.qIKSolVec.at(0),
      ik.qIKSolVec.at(1), ik.qIKSolVec.at(2),
      ik.qIKSolVec.at(3), ik.qIKSolVec.at(4),
      ik.qIKSolVec.at(5);

  printf("\nIK solution vector initialized:");
  printVector(ik.qIKSolVec, 0);

  QVector<Tum::VectorDOFd> currentGoalOutput;
  // Times used as input for trajectory calculation
  ros::Time initialTime;
  ros::Duration passedTime;
  ros::Time endTime;

  initialTime = ros::Time::now();
  int numberPublishes = (int)floor(minInitDuration / pubIntf.jointPubCycleTime);
  printf("\nminInitDuration in secs : %f", minInitDuration);
  printf("\ninitialTime in secs : %f", initialTime.toSec());
  printf("\nendTime in secs : %f", endTime.toSec());
  printf("\nnumber Publishes : %d", numberPublishes);

  // Calculate smooth trajectory from default robot position to hand position
  // and publish it iteratively to robot
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  for (int i = 0; i <= numberPublishes;
       ++i) {  // publish one more time to publish goalPosition
    if (node.ok()) {
      passedTime = ros::Time::now() - initialTime;
      // GetJointPVT5 calculates a smooth trajectory and returns it stepwise
      currentGoalOutput = Tum::Tools::MathTools::getJointPVT5(
          defaultPosInput, goalPosInput, passedTime.toSec(), minInitDuration);

      // Type conversion from QVector<Tum::VectorDOFd> to std::vector<double>
      std::copy_n(&(currentGoalOutput[0](0)),
                  currentGoalOutput[0].size(), currentGoal.begin());

      // Publish Data to Robot
      int success = -1;
      success = checkAndPublishDesJointValues(node, currentGoalRef, lastPubPosition,
                                    pubIntf, pubIntf.jointPubCycleTime, robRestr);
      if (success == - 1){
        printf("Broke velocity restr. during init. Wait until end and reinitialize");
      }
      // check if published goal is already reached. necessary because,
      // checkAndPublishDesJointValues needs more time than pubIntf.jointPubCycleTime
      int equalJoints = 0;
      for (int i = 0; i < lastPubPosition.size(); ++i) {
        if (lastPubPosition.at(i) == goalPosInput[i]) {
          equalJoints++;
        }
      }
      if (equalJoints == 6) {
        printf(
            "\nDefault position has been reached. You can start moving your "
            "hand slowly now.");
        break;
      };
    } else {
      printf("\nAborted initialization process. ");
      ros::shutdown();
    }
  }
  passedTime = ros::Time::now() - initialTime;
  printf("\npassedTime in secs : %f", passedTime.toSec());
}

// Sets the default robot position. Simulated robot: standard parking position;
// Real robot: Position published by robot controller.
// returns -1 for failure, 0 for success
int getDefaultRobotPosition(std::vector<double>& robotPosition,
                            bool publishToController){
  // Simulation mode:
  if (publishToController == false) {
    std::vector<double> defaultSimuPos = {0, -M_PI/2, 0, -M_PI/2, 0, 0};
    robotPosition = defaultSimuPos;
    printf("\nStarting in simulation mode with parking position. "
           "Do NOT connect real robot!");
  }
  // Retrieve real robot position:
  else {
    std::string robotStateTopic = "/ur10_arm_joint_states";
    printf("\nGet default position of real robot from topic "
           "%s", robotStateTopic.c_str());

    // Create shared_ptr to check on received default Position message and
    // not cause an assertion error.
    boost::shared_ptr<sensor_msgs::JointState const> defaultPosMsgPtr;
    defaultPosMsgPtr = ros::topic::waitForMessage<sensor_msgs::JointState>(
                robotStateTopic, ros::Duration(1));
    if( defaultPosMsgPtr == NULL ){
      printf("\nDefault robot position is not published. Start "
             "tum_ics_ur_robot_controller or active simuMode and "
             "set publishToController to false. Retrying.");
      return -1;
    } else {
      sensor_msgs::JointState robotPositionMsg;
      robotPositionMsg = *defaultPosMsgPtr;
      robotPosition = robotPositionMsg.position;
    }
  }
  // Get user confirmation for initial position:
  printf("\nDefault position of robot: ");
  printVector(robotPosition, 0);
  printf(
      "\nIs this the current robot position in deg.? Are you ABSOLUTELY sure to "
      " publish it? y or n?\n");
  char response;
  cin >> response;
  printf("\nthis  was your response: %c", response);
  if ((response == 'Y' || response == 'y')) {
    printf("\nShown default position is assumed to be correct.\n");
    return 0;
  } else {
    printf("\nTrying to get updated position.");
    return -1;
  }
}

int main(int argc, char** argv) {

  // Declare general variables
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  ros::init(argc, argv, "follow_hand_node");

  ros::NodeHandle node;
  ros::NodeHandle& nodeRef = node;
  int loopCounter = 0;
  int success;  // int to check for successful execution of functions

  // PercNeuron runs at 60Hz for 19-32 connected IMUs, at 120Hz for 18 or less.
  // All modules in between run at 60Hz. 120Hz cause instabilities in the rate.
  // The rate can be adapted to 120Hz by changing the parameters in
  // perc_neuron_tf_broadcaster and PercNeuronROSserial
  double framesPerSecPercNeuron = 60;

  double mainCycleTime = 1 / framesPerSecPercNeuron;

  // Initialize Perception Neuron Interface
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  struct tfPercNeuronInterface tfPNIntf;  // Interface to perc.neuron tf data
                                          // created by perc neuron tf
                                          // broadcaster
  tfPNIntf.transBaseLinkRightHand = tf::Transform(
      tf::Quaternion(0, 0, 0, 1),
      tf::Vector3(0, 0, 0));  // Transform from base_link to RightHand directly

  tfPNIntf.tfBroadcaster.sendTransform(
      tf::StampedTransform(tfPNIntf.transBaseLinkRightHand, ros::Time::now(),
                           "/base_link", "/baseLinkToHand"));

  // Create string array with the joints in order of the skeleton hierarchy,
  // provided by the Perception Neuron API, to calculate goal transform
  tfPNIntf.bodyJointsRightHand = {
      "Hips", "Spine", "Spine1", "Spine2", "Spine3", "Neck",
      "RightShoulder", "RightArm", "RightForeArm", "RightHand"};

  struct tfPercNeuronInterface& tfPNIntfRef = tfPNIntf;

  // Declare interface to UR10 / publisher for joint values
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // All ur# values, including DH (Denavit-Hartenberg) values are stored on the
  // parameter server
  // on $ rosparam get /robot_description
  struct pubIntf pubIntf;  // Create interface for publishing joint values to
                           // controller/robot
  pubIntf.jointPubCycleTime = 0.008;  // Cycle time of output joints

  // Set output topic
  // Publishes to the tum_ics_ur_robot_controller
  std::string realRobotTopic = "/joint_desired_cmd";
  // Publishes directly to robot simulation passes the controller
  std::string simRobotTopic = "/ur10_arm_joint_states";
  pubIntf.publishToController = true;
  nodeRef.getParam("/follow_hand/publishToController", pubIntf.publishToController);
  if (pubIntf.publishToController){
    printf("\nPublish data values to robot controller on topic: "
           "%s", realRobotTopic.c_str());
    pubIntf.jointPosPublisher = node.advertise<sensor_msgs::JointState>(
        realRobotTopic,
        1000);
  }
  else {
    printf("\nATTENTION! Do NOT connect real robot! Confirm to start in simulation "
           "mode and publish on %s: y. Terminate: n and set /publishToController on "
           "true. y or n?\n", simRobotTopic.c_str());
    char response;
    cin >> response;
    printf("\nresponse: %c", response);
    if ((response == 'Y' || response == 'y')) {
      pubIntf.jointPosPublisher = node.advertise<sensor_msgs::JointState>(
        simRobotTopic,1000);
    }
    else {
        printf("\nTerminating node. Restart in real robot mode.");
        ros::shutdown();
    }
  }

  // Initialize joint state message
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // Designed for robot joint limited version!, if another is desired: change
  // calculateTraj.
  pubIntf.jointMsg.header.stamp = ros::Time::now();

  pubIntf.jointMsg.header.frame_id = "";

  std::vector<std::string> jointNames = {
      "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
      "wrist_1_joint",      "wrist_2_joint",       "wrist_3_joint"};
  pubIntf.jointMsg.name = jointNames;
  pubIntf.jointMsg.velocity = {};
  pubIntf.jointMsg.effort = {};

  struct pubIntf& pubIntfRef = pubIntf;

  // Set physical restrictions to Robot
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  struct robotRestrictions robRestr;
  robRestr.checkElbowRestriction;

  double innerElbowMinDeg = 20;  // = M_PI/9;
  nodeRef.getParam("/follow_hand/qInnerElbowMin", innerElbowMinDeg);
  robRestr.innerElbowMin = innerElbowMinDeg / 180 * M_PI;  // convert deg to rad

  double innerElbowMaxDeg = 160;
  nodeRef.getParam("/follow_hand/qInnerElbowMax", innerElbowMaxDeg);
  robRestr.innerElbowMax = innerElbowMaxDeg / 180 * M_PI;

  double maxVelDegPSec = 17;  // default maxVel = 17°/sec = 17/180*M_PI rad/sec
                              // = 0.29670597283 rad/sec
  node.getParam("/follow_hand/maxVel", maxVelDegPSec);
  printf("\nMaximum velocity in degree per seconds: %f", maxVelDegPSec);
  robRestr.maxVel = maxVelDegPSec / 180 * M_PI;

  const struct robotRestrictions& robRestrRef = robRestr;

  // Get the defaultRobotPosition and set it as defaultRobotPosition
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  std::vector<double> defaultRobotPosition;
  std::vector<double>& defaultRobotPositionRef = defaultRobotPosition;
  success = -1;
  while (success == -1 && node.ok()) {
    success = getDefaultRobotPosition(defaultRobotPositionRef,
                                      pubIntf.publishToController);
    ros::Duration(2).sleep();
  }

  // Initialize variables for calculation of inverse kinematics
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  std::vector<double> lastPubPosition =
      defaultRobotPositionRef;  // last to robot published joint position
  std::vector<double>& lastPubPositionRef = lastPubPosition;
  tf::Matrix3x3 rotMatrix;

  struct ik ik;
  ik.transForIK = new double[16];  // 4x4 transform matrix in workspace, input
                                   // for ik calculations
  ik.qIKSolMat =
      new double[8 * 6];  // 8 possible ik solutions * 6 joints in joint space
  /**  table for solutions:
       shoulder left = on left side of body (from eyes of perception neuron
     person) and shoulder_pan_joint > 0?
       i = 0 : elbow up;    shoulder left;      wrist up / away from body
       i = 1 : elbow down;  shoulder left;      wrist up / away from body
       i = 2 : elbow up;    shoulder left;      wrist down / at body;
         = 2 : e.g.: -0.713767 -0.823703 1.703598 2.607962 -1.540564 -2.063133
       i = 3 : elbow down;  shoulder left;      wrist down / at body
       i = 4 : elbow down;  shoulder right;     wrist down / at body
       i = 5 : elbow up;    shoulder right;     wrist at down / body;
         = 5 : e.g.:  2.805665 -2.314047 -1.709266 0.515700 1.246372 -1.930651
       i = 6 : elbow down;  shoulder right;     wrist up / away from body
       i = 7 : elbow up;    shoulder right;     wrist up / away from body
  */
  ik.ikSolIndex = 5;
  ik.qIKSolVec = defaultRobotPositionRef;
  struct ik& ikRef = ik;

  // Drive Robot from Robot Default Position to Perc. Neuron Default Position
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  printf("\nInitialize Robot for the first time.");
  printf("\nDefault Position: ");
  printVector(defaultRobotPositionRef, 0);
  robRestr.checkElbowRestriction = 0;  // Elbow restrictions are only checked at
          // the beginning of initialization process if it's possible to reach
          // goal position, because robot default position can be out of the
          // restriction boundaries

  success = -1;
  printf("\nStarting initialization\n");
  success = initializeRobotToHand(nodeRef, pubIntfRef, tfPNIntfRef, ikRef,
                                  lastPubPositionRef, defaultRobotPositionRef,
                                  robRestrRef);
  if(success == -1){
    printf("\nInitialization process failed. Shutdown ros.\n");
    ros::shutdown();
  }

  robRestr.checkElbowRestriction = 1;

  // Start the loop to get tf position from Perception Neuron and publish it to the
  // robot / controller
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  printf("\nYou can start moving slowly now.");
  ros::Time begin = ros::Time::now();
  ros::Duration elapsedTime = begin - begin;
  ros::Rate rate(framesPerSecPercNeuron);
  while (node.ok()) {
    begin = ros::Time::now();

    // Get goal transform for UR10's end effector from UR10's base_link to
    // Perception Neuron's RightHand
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    success = -1;
    while (success == -1) {
      success = tfPNIntfRef.getTransBaseLinkRightHand();
    }

    // Get Goal Joint Positions from transBaseLinkRightHand by Inverse Kinematics
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ik.calcqIKSolVec(tfPNIntfRef);  // sets ik.qIKSolVec

    // Check physical restrictions, interpolate points from rate framesPerSecPercNeuron
    // to max 125HZ = 1frame / 8ms and publish them to the controller / robot
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    elapsedTime = ros::Time::now() - begin;

    success = checkAndPublishDesJointValues(nodeRef, ik.qIKSolVec,
                                                lastPubPositionRef, pubIntfRef,
                                                mainCycleTime, robRestrRef);

    // Reinitialization if goal position couldn't be published
    // e.g. If human controller moved to fast or out of range
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    char reinitializeResponse;
    if (-1 == success) {  // robot desired value couldn't be published
      printf("\nCurrent Robot Position / lastPubPosition: ");
      printVector(lastPubPositionRef, 0);
      printf(
          "\nDesired goal could not be published.\nDo you want to reinitialize "
          "the robot, so that it drives to your arm?\ny or n?\n");
      cin >> reinitializeResponse;
      printf("\nthis  was your response: %c", reinitializeResponse);
      if ((reinitializeResponse != 'Y' && reinitializeResponse != 'y')) {
        printf("\nAre you sure to terminate the program?\ny or n?\n");
        cin >> reinitializeResponse;
        if ((reinitializeResponse == 'Y' || reinitializeResponse == 'y')) {
          printf("\nTerminate Node.");
          ros::shutdown();
          return 0;
        }
      } else {
        printf("\nStart Reinitialization. Please hold your hand steady.");
        // now defaultRobotPosition is lastPubPosition
        robRestr.checkElbowRestriction = 0;  // Elbow restrictions are 
                // only checked at the beginning ofs
                // initialization process if it's possible to reach goal
                // position, because robot default position can be out of the
                // restriction boundaries
        success = -1;
        while (success == -1 && node.ok()) {
          success = initializeRobotToHand(nodeRef, pubIntfRef, tfPNIntfRef, ikRef,
                             lastPubPositionRef, lastPubPositionRef, robRestrRef);
          if(success == -1){
            printf("\nReinitialization process failed. Trying again in 1sec.\n");
            ros::Duration(1).sleep();
          }
        }
        robRestr.checkElbowRestriction = 1;
      }
    }
    loopCounter++;
    rate.sleep();
  }
  ros::shutdown();
  return 0;
}
