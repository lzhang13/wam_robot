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

sensor_msgs::JointState currJointState;
void wamJointStateCallback(const sensor_msgs::JointState::ConstPtr& jointState)
{
    /*ROS_INFO("IN wamJointStateCallback");
  	ROS_INFO("J[%4.2f, %4.2f, %4.2f, %4.2f, %4.2f, %4.2f, %4.2f]", jointState->position[0],jointState->position[1],
  	jointState->position[2],jointState->position[3],jointState->position[4],jointState->position[5],jointState->position[6]); */
  	currJointState = *jointState;
}

//Subscribe to the biotac output and obtain force reading(changes made by Nancy)
void biotacCallback(const biotac_sensors::BioTacHand& msg)
{
  /*
    int pdc(msg.bt_data[1].pdc_data);
    ROS_INFO("PDC = %d", pdc);
    ros::NodeHandle n("sub"); // Lionel
    ros::ServiceClient hold_joint_pos = n.serviceClient<wam_msgs::Hold>("hold_joint_pos");
    wam_msgs::Hold holdRequest;
    holdRequest.request.hold = false;
    if(msg.bt_data[0].pdc_data > 2200)
       {
          ROS_INFO("PDC THRESHOLD EXCEEDED");
          if(hold_joint_pos.call(holdRequest))
          {
            ROS_INFO("WAM STOPPED");
          }
       }
    for(int i = 0; i < 19; i++)
    {
        if(msg.bt_data[0].electrode_data[i] < 1700)
        {
          ROS_INFO("ELECTRODE %d THRESHOLD EXCEEDED", i+1);
          if(hold_joint_pos.call(holdRequest))
          {
            ROS_INFO("WAM STOPPED");
          }
        }
    }
    */
}

wam_msgs::JointTrajectoryVelocityMove wamMove(float start[], float end[])
{
  wam_msgs::JointTrajectoryVelocityMove jointTrajectoryMove;
  trajectory_msgs::JointTrajectory jointTrajectory;
  trajectory_msgs::JointTrajectoryPoint jointTrajectoryPoint1, jointTrajectoryPoint2;

  jointTrajectoryMove.request.length = 2.0; //travel length of end effector
  jointTrajectoryMove.request.velocity = 0.05; // 0.02 m/s or 2 cm/s

  for (size_t i = 0; i < 7; i++)
  {
      ROS_INFO("****DEBUG_3333***************");
      jointTrajectoryPoint1.positions.push_back(start[i]);
      jointTrajectoryPoint2.positions.push_back(end[i]);
  }
  jointTrajectory.points.push_back(jointTrajectoryPoint2);
  jointTrajectory.points.push_back(jointTrajectoryPoint1);
  
  jointTrajectoryMove.request.jointTrajectory = jointTrajectory;
  return jointTrajectoryMove;
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "wam_bt_force");
  ros::NodeHandle nh("bhand"); 
	ros::NodeHandle nw;
  
  sleep(0.01);


	// WAM AND BHAND SERVICES AND INITIAL MOVEMENTS (NO TRAJECTORY PLANNING)
  	//Barrett_WAM interface
  	//Subscribers
	ros::Subscriber wam_joint_state_sub = nw.subscribe("joint_states",1000, wamJointStateCallback);
	//Services see wam_node for additional msgs and srv
	//bhand
	ros::ServiceClient close_grasp  = nh.serviceClient<std_srvs::Empty>("close_grasp");
	ros::ServiceClient open_grasp   = nh.serviceClient<std_srvs::Empty>("open_grasp");
	ros::ServiceClient close_spread = nh.serviceClient<std_srvs::Empty>("close_spread");
	ros::ServiceClient open_spread = nh.serviceClient<std_srvs::Empty>("open_spread");
	ros::ServiceClient finger_pos   = nh.serviceClient<wam_msgs::BHandFingerPos>("finger_pos");
	ros::ServiceClient spread_pos   = nh.serviceClient<wam_msgs::BHandSpreadPos>("spread_pos");

	//wam
	ros::ServiceClient joint_move  = nw.serviceClient<wam_msgs::JointMove>("joint_move");
	ros::ServiceClient go_home = nw.serviceClient<std_srvs::Empty>("go_home");
	ros::ServiceClient joint_trajectory_velocity_move  = nw.serviceClient<wam_msgs::JointTrajectoryVelocityMove>("joint_trajectory_velocity_move");
	ros::ServiceClient biotac_logger = nw.serviceClient<wam_msgs::LoggerInfo>("logger_controller");
	ros::ServiceClient special_service = nw.serviceClient<wam_msgs::SpecialService>("special_service");
  ros::ServiceClient hold_joint_pos = nw.serviceClient<wam_msgs::Hold>("hold_joint_pos"); //change made by Nancy

	std_srvs::Empty empty_srv_1, empty_srv_2;
  //wam_msgs::Hold holdjointposition; //change made by Nancy

  float home[] = {-1.5859, -1.6596, 0.0026, 0.1787, 0.1107, 0.2503, 0};
  float test[] = {-1.5859, -1.6596, 0.4026, 0.1787, 0.1107, 0.2503, 0};
  float standby[] = {-0.0487222, -1.3306, 1.62575, 2.04923, -0.0101211, -0.378435, 0.0185968};
  float standby2[] = {-0.0487222, -1.3306, 1.62575, 2.04923, -0.0101211, -0.378435, 0.0185968-0.3};
  float dig_in_a[] = {-0.491787, -1.41935, 1.71565, 1.47543, 0.0227725, -0.330043, -0.396903};

  wam_msgs::JointTrajectoryVelocityMove jtm;
  std::cout << "[Enter] to go to standby";
  while(getchar() != '\n');
  jtm = wamMove(home,standby);

  if(joint_trajectory_velocity_move.call(jtm))
      ROS_INFO("JointTrajectoryVelocityMove success status = %lu",jtm.response.status);
  else
      ROS_WARN("JointTrajectoryVelocityMove FAIL ######");

  /*
  wam_msgs::JointTrajectoryVelocityMove jointTrajectoryMove;
  trajectory_msgs::JointTrajectory jointTrajectory;
  trajectory_msgs::JointTrajectoryPoint jointTrajectoryPoint1, jointTrajectoryPoint2;

  jointTrajectoryMove.request.length = 2.0; //travel length of end effector
  jointTrajectoryMove.request.velocity = 0.05; // 0.02 m/s or 2 cm/s

  for (size_t i = 0; i < 7; i++)
  {
      ROS_INFO("****DEBUG_3333***************");
      jointTrajectoryPoint1.positions.push_back(home[i]);
      jointTrajectoryPoint2.positions.push_back(standby[i]);
  }
  jointTrajectory.points.push_back(jointTrajectoryPoint1);
  jointTrajectory.points.push_back(jointTrajectoryPoint2);

  std::cout << "[Enter] to go to standby";
  while(getchar() != '\n');
  
  jointTrajectoryMove.request.jointTrajectory = jointTrajectory;
  if(joint_trajectory_velocity_move.call(jointTrajectoryMove))
      ROS_INFO("JointTrajectoryVelocityMove success status = %lu",jointTrajectoryMove.response.status);
  else
      ROS_WARN("JointTrajectoryVelocityMove FAIL ######");

    */
    /*
  wam_msgs::Hold test;
  test.request.hold = 0;

  if(hold_joint_pos.call(test))
    ROS_INFO("HOLD SUCCESS");
  else 
    ROS_WARN("HOLD FAIL");
    */
  /*
  while(true)
  {
      // Ask user for mode
      char user_input;
      std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~\n";
      std::cout << "X: DIG W/O CONTROL\n";

      std::cout << "Enter desired mode: ";
      std::cin >> user_input;

      

      if (user_input == 'X')
      {
          float dig_out[] = {-0.355884, -1.39864, 1.69444, 1.7305, 0.0663407, -0.458692, -0.397212};
          float dig_in_a[] = {-0.491787, -1.41935, 1.71565, 1.47543, 0.0227725, -0.330043, -0.396903}; //
          //float dig_in_b[] = {-0.597814, -1.38281, 1.77988, 1.78027, 0.0386658, -0.586155, -0.399472};
          float dig_in_c[] = {-0.799642, -1.31445, 1.73819, 1.96903, -0.100974, -0.903863, -0.399267};
          
          std::cout << "\n\n\n\n\n~~~~~~DIG MODE~~~~~~\n\n";

          std::cout << "[Enter] to move to dig_out\n";
          while(getchar() != '\n');
          for (size_t i = 0; i < 7; i++)
              jp_wam[i] = dig_out[i];
          wam.moveTo(jp_wam);

          std::cout << "[Enter] to move to dig_in_a\n";
          while(getchar() != '\n');
          for (size_t i = 0; i < 7; i++)
              jp_wam[i] = dig_in_a[i];
          wam.moveTo(jp_wam);

          std::cout << "[Enter] to move to dig_in_c\n";
          while(getchar() != '\n');
          for (size_t i = 0; i < 7; i++)
          {
              ROS_INFO("****DEBUG_3333***************");
              jointTrajectoryPoint1.positions.push_back(dig_in_a[i]);
              jointTrajectoryPoint2.positions.push_back(dig_in_c[i]);
          }
          jointTrajectory.points.push_back(jointTrajectoryPoint1);
          jointTrajectory.points.push_back(jointTrajectoryPoint2);
          
          jointTrajectoryMove.request.jointTrajectory = jointTrajectory;
          if(joint_trajectory_velocity_move.call(jointTrajectoryMove))
              ROS_INFO("JointTrajectoryVelocityMove success status = %lu",jointTrajectoryMove.response.status);
          else
              ROS_WARN("JointTrajectoryVelocityMove FAIL ######");
      }
  }
  
  */
	sleep(10.0);
  
  ROS_INFO("I MADE IT TO THE END!!!");
  	
	ros::waitForShutdown();
	//ros::shutdown();  
	return 0;
}

