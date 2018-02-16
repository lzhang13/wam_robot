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
float currJS[7];
void wamJointStateCallback(const sensor_msgs::JointState::ConstPtr& jointState)
{
	//ROS_INFO("IN wamJointStateCallback");
	/*
	ROS_INFO("J[%4.2f, %4.2f, %4.2f, %4.2f, %4.2f, %4.2f, %4.2f]", jointState->position[0],jointState->position[1],
	jointState->position[2],jointState->position[3],jointState->position[4],jointState->position[5],jointState->position[6]); */
	currJointState = *jointState;
	for (size_t i = 0; i < 7; i++)
		currJS[i] = currJointState.position[i];
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

wam_msgs::JointTrajectoryVelocityMove wamMove(float end[], float deltaTime)
{
	wam_msgs::JointTrajectoryVelocityMove jointTrajectoryMove;
	trajectory_msgs::JointTrajectory jointTrajectory;
	trajectory_msgs::JointTrajectoryPoint jointTrajectoryPoint1, jointTrajectoryPoint2;

	jointTrajectoryMove.request.length = deltaTime; //travel length of end effector
	jointTrajectoryMove.request.velocity = 0.5; // 0.02 m/s or 2 cm/s
	ros::spinOnce();

	for (size_t i = 0; i < 7; i++)
	{
		ROS_INFO("****DEBUG_3333***************");
		jointTrajectoryPoint1.positions.push_back(currJS[i]);
		jointTrajectoryPoint2.positions.push_back(end[i]);
	}
	jointTrajectory.points.push_back(jointTrajectoryPoint1);
	jointTrajectory.points.push_back(jointTrajectoryPoint2);
	
	jointTrajectoryMove.request.jointTrajectory = jointTrajectory;

	printf("[Enter] to move to %s\n",end);
	while(getchar() != '\n');

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
	ros::ServiceClient joint_trajectory_velocity_move_bt  = nw.serviceClient<wam_msgs::JointTrajectoryVelocityMove>("joint_trajectory_velocity_move_bt");
	ros::ServiceClient biotac_logger = nw.serviceClient<wam_msgs::LoggerInfo>("logger_controller");
	ros::ServiceClient special_service = nw.serviceClient<wam_msgs::SpecialService>("special_service");
	ros::ServiceClient hold_joint_pos = nw.serviceClient<wam_msgs::Hold>("hold_joint_pos"); //change made by Nancy

	std_srvs::Empty empty_srv_1, empty_srv_2;
	wam_msgs::JointMove jp_wam;
	bool demo_mode = false;

	float home[] = {-1.5859, -1.6596, 0.0026, 0.1787, 0.1107, 0.2503, 0};
	float test[] = {-1.5859, -1.6596, 0.4026, 0.1787, 0.1107, 0.2503, 0};
	float standby[] = {-0.0487222, -1.3306, 1.62575, 2.04923, -0.0101211, -0.378435, 0.0185968};
	float standby2[] = {-0.0487222, -1.3306, 1.62575, 2.04923, -0.0101211, -0.378435, 0.0185968-0.3};
	float dig_out[] = {-0.355884, -1.39864, 1.69444, 1.7305, 0.0663407, -0.458692, -0.397212};

	for (size_t i = 0; i < 7; i++)
		jp_wam.request.joints.push_back(0.0);

	wam_msgs::JointTrajectoryVelocityMove jtm;

	while(true)
	{
		// Ask user for mode
		char user_input;
		std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~\n";
		std::cout << "S: GO STANDBY\n";
		std::cout << "X: DIG W/O CONTROL\n";
		std::cout << "C: SEARCH\n";
		std::cout MM "V: LOCATE\n";
		std::cout << "H: GO HOME\n\n";

		std::cout << "I: IDLE MODE\n";
		std::cout << "l: Read joint angles\n\n";

		std::cout << "D: TOGGLE DEMO MODE\n";
        if (demo_mode) std::cout << "DEMO MODE ON\n";
        else std::cout << "DEMO MODE OFF\n\n";

		std::cout << "Enter desired mode: ";
		std::cin >> user_input;
		while(getchar() != '\n');

		if (user_input == 'S')
		{
			while(getchar() != '\n');
			std::cout << "[Enter] to go to standby";
			while(getchar() != '\n');
			for (size_t i = 0; i < 7; i++)
				jp_wam.request.joints[i] = standby[i];
			joint_move.call(jp_wam);
		}

		if (user_input == 'X')
		{
			float dig_in_a[] = {-0.491787, -1.41935, 1.71565, 1.47543, 0.0227725, -0.330043, -0.396903}; 
			float dig_in_c[] = {-0.799642, -1.31445, 1.73819, 1.96903, -0.100974, -0.53863, -0.399267};
			
			std::cout << "\n\n\n\n\n~~~~~~DIG MODE~~~~~~\n\n";
			for (int i = 0; i < 2; i++)
			{
				std::cout << "[Enter] to move to dig_out\n";
				while(getchar() != '\n');
				for (size_t i = 0; i < 7; i++)
					jp_wam.request.joints[i] = dig_out[i];
				joint_move.call(jp_wam);

				std::cout << "[Enter] to move to dig_in_a\n";
				while(getchar() != '\n');
				for (size_t i = 0; i < 7; i++)
					jp_wam.request.joints[i] = dig_in_a[i];
				joint_move.call(jp_wam);

				//std::cout << "[Enter] to move to dig_in_c\n";
				//while(getchar() != '\n');
				jtm = wamMove(dig_in_c,6.0);
				if(joint_trajectory_velocity_move_bt.call(jtm))
					ROS_INFO("JointTrajectoryVelocityMoveBt success status = %lu",jtm.response.status);
				else
					ROS_WARN("JointTrajectoryVelocityMoveBt FAIL ######");  
			}
		}

		if (user_input == 'C')
		{
			float dig_in_1a[] = {-0.491787, -1.41935, 1.71565, 1.47543, 0.0227725, -0.330043, -0.396903}; 
			float dig_in_1b[] = {-0.799642, -1.31445, 1.73819, 1.96903, -0.100974, -0.53863, -0.399267};
			
			std::cout << "\n\n\n\n\n~~~~~~SEARCH MODE~~~~~~\n\n";

			for (int i = 0; i < 2; i++)
			{
				std::cout << "[Enter] to move to dig_out\n";
				while(getchar() != '\n');
				for (size_t i = 0; i < 7; i++)
					jp_wam.request.joints[i] = dig_out[i];
				joint_move.call(jp_wam);

				std::cout << "[Enter] to move to dig_in_1a\n";
				while(getchar() != '\n');
				for (size_t i = 0; i < 7; i++)
					jp_wam.request.joints[i] = dig_in_1a[i];
				joint_move.call(jp_wam);

				std::cout << "[Enter] to move to dig_in_1b\n";
				while(getchar() != '\n');
				jtm = wamMove(dig_in_1a,dig_in_1b,10.0);
				if(joint_trajectory_velocity_move.call(jtm))
					ROS_INFO("JointTrajectoryVelocityMove success status = %lu",jtm.response.status);
				else
					ROS_WARN("JointTrajectoryVelocityMove FAIL ######");

				ros::spinOnce();
				std::cout << "[Enter] to move to dig_out\n";
				while(getchar() != '\n');
				jtm = wamMove(currJS,dig_out,6.0);
				if(joint_trajectory_velocity_move_bt.call(jtm))
					ROS_INFO("JointTrajectoryVelocityMoveBt success status = %lu",jtm.response.status);
				else
					ROS_WARN("JointTrajectoryVelocityMoveBt FAIL ######"); 
			}
		}

		if (user_input == 'V')
		{
			float end_to_out = 6;
			float out_to_start = 4;
			float start_to_end = 10;
			
			float dig_1out[] = {};
			float dig_1start[] = {};
			float dig_1end[] = {};
			float dig_2out[] = {};
			float dig_2start[] = {};
			float dig_2end[] = {};
			float dig_3out[] = {};
			float dig_3start[] = {};
			float dig_3end[] = {};
			float dig_4out[] = {};
			float dig_4start[] = {};
			float dig_4end[] = {};
			float dig_5out[] = {};
			float dig_5start[] = {};
			float dig_5end[] = {};
			
			jtm = wamMove(currJS,dig_1out,end_to_out);
			joint_trajectory_velocity_move_bt.call(jtm)
			jtm = wamMove(currJS,dig_1start,out_to_start);
			joint_trajectory_velocity_move_bt.call(jtm)
			jtm = wamMove(currJS,dig_1end,start_to_end);
			joint_trajectory_velocity_move.call(jtm)


		}

		if (user_input == 'H')
		{
			std::cout << "[Enter] to go to Home";
			while(getchar() != '\n');
			for (size_t i = 0; i < 7; i++)
				jp_wam.request.joints[i] = home[i];
			joint_move.call(jp_wam); 
		}

		if (user_input == 'I')
		{
			std::cout << "[Enter] to idle\n";
		    while(getchar() != '\n');
		    wam_msgs::Hold holdRequest;
			holdRequest.request.hold = false;
			hold_joint_pos.call(holdRequest);
			
		}
		if (user_input == 'l')
		{
			ros::spinOnce();
			for (size_t i = 0; i < 7; i++)
			{
				std::cout << currJS[i];
				if (i < 6) std::cout << ", ";                        
			}
		}
	}
	
	sleep(10.0);
	
	ROS_INFO("I MADE IT TO THE END!!!");
	ros::waitForShutdown();
	//ros::shutdown();  
	return 0;
}