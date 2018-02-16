#include <ros/ros.h>
#include <ros/package.h>
#include <stdio.h>
#include <stdlib.h>
#include <error.h>
#include <unistd.h>
#include <string.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <cstdlib>

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

//Barrett_WAM msgs
#include "wam_msgs/RTJointPos.h"
//Barrett_WAM srvs
#include "wam_msgs/JointMove.h"
#include "wam_msgs/JointTrajectoryVelocityMove.h"
#include <wam_msgs/LoggerInfo.h>
#include <wam_msgs/SpecialService.h>
#include <wam_msgs/BHandFingerPos.h>
#include <wam_msgs/BHandSpreadPos.h>
#include <wam_msgs/Hold.h> // Lionel

//ROS standard messages
#include "std_srvs/Empty.h"
#include "sensor_msgs/JointState.h"
#include "math.h"
//#include "biomechHelper.h"
//#include "HMMFunctional.h"

//BioTac messages
//#include <biotac_sensors/biotac_hand_class.h>
#include <biotac_sensors/BioTacHand.h> // Lionel

int main(int argc, char **argv)
{

  ros::init(argc, argv, "test_client");
  ros::NodeHandle nw("client");


  ros::ServiceClient hold_joint_pos = nw.serviceClient<wam_msgs::Hold>("hold_joint_pos"); //change made by Nancy
  
  wam_msgs::Hold test;
  test.request.hold = 0;
  std::cout << "[Enter] to go to standby";
  ROS_INFO("~~~~~~DEBUG 1111~~~~~~");
  while(getchar() != '\n');

  if(hold_joint_pos.call(test))
    ROS_INFO("HOLD SUCCESS");
  else 
    ROS_WARN("HOLD FAIL");
  ROS_INFO("~~~~~~DEBUG 2222~~~~~~");
  sleep(5.0);
  
  ROS_INFO("I MADE IT TO THE END!!!");
    
  //ros::waitForShutdown();
  ros::shutdown();  
  return 0;
}
