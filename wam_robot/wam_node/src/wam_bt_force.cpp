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
bool demo_mode = false;
char defaultFilename[] = "/etc/barrett/bus1/calibration_data/wam7w/gravitycal.conf";
char robotiqFilename[] = "/etc/barrett/bus1/calibration_data/wam7w/gravitycal.conf.robotiq";
char barrettFilename[] = "/etc/barrett/bus1/calibration_data/wam7w/gravitycal.conf.barrett";

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

bool fexists(const char *filename)
{
	std::ifstream ifile(filename);
	return ifile;
}

void switchConfig()
{
	// Switch from barrett to robotiq
	if (fexists(robotiqFilename) && !fexists(barrettFilename))
	{
		int result = rename(defaultFilename, barrettFilename);
		if (result == 0)
		{
			std::cout << "Stored barrett grav config\n";
			result = rename(robotiqFilename, defaultFilename);
			if (result == 0)
				std::cout << "Using robotiq grav config\n";
			else
				std::cout << "Failed using robotiq grav config\n";
		}
		else
			std::cout << "Failed storing barrett grav config\n";
	}
	// Switch from robotiq to barrett
	else if (fexists(barrettFilename) && !fexists(robotiqFilename))
	{
		int result = rename(defaultFilename, robotiqFilename);
		if (result == 0)
		{
			std::cout << "Stored robotiq grav config\n";
			result = rename(barrettFilename, defaultFilename);
			if (result == 0)
				std::cout << "Using barrett grav config\n";
			else
				std::cout << "Failed using barrett grav config\n";
		}
		else
			std::cout << "Failed storing robotiq grav config\n";
	}
}

void showCurrentPos()
{
	ros::spinOnce();
	for (size_t i = 0; i < 7; i++)
	{
		std::cout << currJS[i];
		if (i < 6) std::cout << ", ";
		if (i == 6) std::cout << "\n\n";                       
	}
}

wam_msgs::JointTrajectoryVelocityMove wamMove(float end[], float deltaTime)
{
	wam_msgs::JointTrajectoryVelocityMove jointTrajectoryMove;
	trajectory_msgs::JointTrajectory jointTrajectory;
	trajectory_msgs::JointTrajectoryPoint jointTrajectoryPoint1, jointTrajectoryPoint2;

	jointTrajectoryMove.request.length = deltaTime; //travel length of end effector
	jointTrajectoryMove.request.velocity = 0.5; // 0.02 m/s or 2 cm/s

	for (size_t i = 0; i < 7; i++)
	{
		jointTrajectoryPoint2.positions.push_back(end[i]);
	}
	ros::spinOnce();
	for (size_t i = 0; i < 7; i++)
	{
		jointTrajectoryPoint1.positions.push_back(currJS[i]);
	}
	jointTrajectory.points.push_back(jointTrajectoryPoint1);
	jointTrajectory.points.push_back(jointTrajectoryPoint2);
	
	jointTrajectoryMove.request.jointTrajectory = jointTrajectory;

	if (!demo_mode) {
		std::cout << "Press [Enter] to continue";
		while(getchar() != '\n');
	}

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
	
	float home[] = {-1.59059, -1.64874, 0.0183817, 0.225069, 0.140193, -0.153319, -0.13552};
	// float home[] = {-1.5859, -1.6596, 0.0026, 0.1787, 0.1107, 0.2503, 0};
	float test[] = {-1.5859, -1.6596, 0.4026, 0.1787, 0.1107, 0.2503, 0};
	float standby[] = {-0.0487222, -1.3306, 1.62575, 2.04923, -0.0101211, -0.378435, 0.0185968};
	float standby2[] = {-0.0487222, -1.3306, 1.62575, 2.04923, -0.0101211, -0.378435, 0.0185968-0.3};
	float dig_out[] = {-0.355884, -1.39864, 1.69444, 1.7305, 0.0663407, -0.458692, -0.397212};

	float moveTime = 10;

	for (size_t i = 0; i < 7; i++)
		jp_wam.request.joints.push_back(0.0);

	wam_msgs::JointTrajectoryVelocityMove jtm;

	// Check grav comp config
	if(fexists(robotiqFilename))
	{
		switchConfig();
		std::cout << "*******USING WRONG CONFIG*******";
	}

	while(true)
	{
		// Ask user for mode
		char user_input;
		std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~\n";
		std::cout << "G: SWITCH GRAV CONFIG\n";
		std::cout << "S: GO STANDBY\n";
		std::cout << "X: DIG W/O CONTROL\n";
		std::cout << "C: SEARCH\n";
		std::cout << "V: LOCATE\n";
		std::cout << "B: SQUARE\n";
		std::cout << "F: FOLLOW PATH\n";
		std::cout << "R: REPEAT MOTION\n";
		std::cout << "P: PLAYBACK\n";
		std::cout << "H: GO HOME\n\n";

		std::cout << "I: IDLE MODE\n";
		std::cout << "l: Read joint angles\n\n";

		std::cout << "D: TOGGLE DEMO MODE\n";
        if (demo_mode) std::cout << "DEMO MODE ON\n";
        else std::cout << "DEMO MODE OFF\n\n";

        if (fexists(robotiqFilename) && !fexists(barrettFilename))
        	std::cout << "USING BARRETT GRAV CONFIG\n\n";
        if (fexists(barrettFilename) && !fexists(robotiqFilename))
        	std::cout << "USING ROBOTIQ GRAV CONFIG\n\n";

		std::cout << "Enter desired mode: ";
		std::cin >> user_input;
		while(getchar() != '\n');

		if (user_input == 'D')
        {
            if (demo_mode) demo_mode = false;
            else demo_mode = true;
        }

        if (user_input == 'G')
        {
        	switchConfig();
        }

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
				jtm = wamMove(dig_in_1b,10.0);
				if(joint_trajectory_velocity_move.call(jtm))
					ROS_INFO("JointTrajectoryVelocityMove success status = %lu",jtm.response.status);
				else
					ROS_WARN("JointTrajectoryVelocityMove FAIL ######");

				ros::spinOnce();
				std::cout << "[Enter] to move to dig_out\n";
				while(getchar() != '\n');
				jtm = wamMove(dig_out,6.0);
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
			
			float dig_4out[] = {-0.433313, -1.02739, 1.96885, 1.66096, -0.609085, -0.348783, -0.235799};
			float dig_4start[] = {-0.611584, -1.0888, 2.07147, 1.39499, -0.564805, -0.302289, -0.255013};
			float dig_4end[] = {-0.695478, -1.03333, 1.97847, 1.63983, -0.562038, -0.327513, -0.289741};
			float dig_3out[] = {-0.430501, -0.95229, 1.95288, 1.54267, -0.587894, -0.295489, 0.463277};
			float dig_3start[] = {-0.603658, -1.00138, 2.0348, 1.31513, -0.630118, -0.2384, 0.421151};
			float dig_3end[] = {-0.707348, -1.0152, 2.05738, 1.64775, -0.707687, -0.349969, 0.275151};
			float dig_2out[] = {-0.534081, -0.771524, 2.04146, 1.69744, -0.731883, -0.367839, 1.03803};
			float dig_2start[] = {-0.706252, -0.84559, 2.16069, 1.46299, -0.70318, -0.333443, 1.02848};
			float dig_2end[] = {-0.725281, -0.966815, 2.21273, 1.63974, -0.669179, -0.43244, 0.95861};
			float dig_1out[] = {-0.634849, -0.645847, 2.06577, 1.90478, -0.759004, -0.517046, 1.69878};
			float dig_1start[] = {-0.850044, -0.740383, 2.22432, 1.60949, -0.796958, -0.476088, 1.84335};
			float dig_1end[] = {-0.784923, -0.853165, 2.41233, 1.57276, -0.780986, -0.467706, 1.89318};
			float dig_5out[] = {-0.378017, -1.34735, 1.64614, 1.83294, -0.908765, -0.58742, -0.327345};
			float dig_5start[] = {-0.564505, -1.37847, 1.69841, 1.52478, -0.914063, -0.528828, -0.23282};
			float dig_5end[] = {-0.67791, -1.24559, 1.56741, 1.62789, -0.998195, -0.581331, -0.328578};
			float dig_6out[] = {-0.520494, -1.2871, 1.63688, 1.9681, -0.950119, -0.705473, -1.02652};
			float dig_6start[] = {-0.673125, -1.31322, 1.68076, 1.7126, -0.955971, -0.692505, -0.955938};
			float dig_6end[] = {-0.693359, -1.23414, 1.54826, 1.62218, -0.972338, -0.67851, -0.969398};
			float dig_7out[] = {-0.738758, -1.23948, 1.59729, 2.07599, -1.01828, -1.07141, -1.74759};
			float dig_7start[] = {-0.838576, -1.27739, 1.66087, 1.79229, -0.95265, -0.920547, -1.67628};
			float dig_7end[] = {-0.68883, -1.19699, 1.526, 1.56159, -1.12937, -0.74319, -1.47932};
			float dig_8out[] = {-0.704645, -1.25713, 1.31805, 1.94415, -1.19761, -1.03615, -2.3583};
			float dig_8start[] = {-0.827656, -1.27999, 1.35646, 1.71005, -1.17223, -1.00602, -2.29789};
			float dig_8end[] = {-0.622979, -1.30041, 1.39076, 1.50961, -1.11783, -0.828587, -2.15435};
			
			jtm = wamMove(dig_1out,20.0);
			joint_trajectory_velocity_move_bt.call(jtm);
			jtm = wamMove(dig_1start,out_to_start);
			joint_trajectory_velocity_move_bt.call(jtm);
			jtm = wamMove(dig_1end,start_to_end);
			joint_trajectory_velocity_move.call(jtm);

			jtm = wamMove(dig_2out,end_to_out);
			joint_trajectory_velocity_move_bt.call(jtm);
			jtm = wamMove(dig_2start,out_to_start);
			joint_trajectory_velocity_move_bt.call(jtm);
			jtm = wamMove(dig_2end,start_to_end);
			joint_trajectory_velocity_move.call(jtm);

			jtm = wamMove(dig_3out,end_to_out);
			joint_trajectory_velocity_move_bt.call(jtm);
			jtm = wamMove(dig_3start,out_to_start);
			joint_trajectory_velocity_move_bt.call(jtm);
			jtm = wamMove(dig_3end,start_to_end);
			joint_trajectory_velocity_move.call(jtm);

			jtm = wamMove(dig_4out,end_to_out);
			joint_trajectory_velocity_move_bt.call(jtm);
			jtm = wamMove(dig_4start,out_to_start);
			joint_trajectory_velocity_move_bt.call(jtm);
			jtm = wamMove(dig_4end,start_to_end);
			joint_trajectory_velocity_move.call(jtm);

			jtm = wamMove(dig_5out,end_to_out);
			joint_trajectory_velocity_move_bt.call(jtm);
			jtm = wamMove(dig_5start,out_to_start);
			joint_trajectory_velocity_move_bt.call(jtm);
			jtm = wamMove(dig_5end,start_to_end);
			joint_trajectory_velocity_move.call(jtm);

			jtm = wamMove(dig_6out,end_to_out);
			joint_trajectory_velocity_move_bt.call(jtm);
			jtm = wamMove(dig_6start,out_to_start);
			joint_trajectory_velocity_move_bt.call(jtm);
			jtm = wamMove(dig_6end,start_to_end);
			joint_trajectory_velocity_move.call(jtm);

			jtm = wamMove(dig_7out,end_to_out);
			joint_trajectory_velocity_move_bt.call(jtm);
			jtm = wamMove(dig_7start,out_to_start);
			joint_trajectory_velocity_move_bt.call(jtm);
			jtm = wamMove(dig_7end,start_to_end);
			joint_trajectory_velocity_move.call(jtm);

			jtm = wamMove(dig_8out,end_to_out);
			joint_trajectory_velocity_move_bt.call(jtm);
			jtm = wamMove(dig_8start,out_to_start);
			joint_trajectory_velocity_move_bt.call(jtm);
			jtm = wamMove(dig_8end,start_to_end);
			joint_trajectory_velocity_move.call(jtm);

			jtm = wamMove(dig_8out,end_to_out);
			joint_trajectory_velocity_move_bt.call(jtm);

			// jtm = wamMove(dig_1out,end_to_out);
			// joint_trajectory_velocity_move_bt.call(jtm);
		}

		if (user_input == 'B')
		{
			moveTime = 6;
			
			float p1[] = {-0.68043, -1.4301, 1.78389, 1.2732, 0.162491, -0.381044, -0.526567};
			float p2[] = {-0.689086, -1.34561, 1.64286, 1.1435, 0.0431729, -0.260777, -0.346045};
			float p3[] = {-0.752856, -1.30893, 1.58123, 1.39575, -0.186134, -0.510009, -0.25193};
			float p4[] = {-0.748948, -1.38634, 1.71109, 1.50552, -0.187715, -0.558401, -0.356936};

			// while(getchar() != '\n');
			// for (size_t i = 0; i < 7; i++)
			// 	jp_wam.request.joints[i] = p1[i];
			// joint_move.call(jp_wam);

			// while(getchar() != '\n');
			// for (size_t i = 0; i < 7; i++)
			// 	jp_wam.request.joints[i] = p2[i];
			// joint_move.call(jp_wam);

			// while(getchar() != '\n');
			// for (size_t i = 0; i < 7; i++)
			// 	jp_wam.request.joints[i] = p3[i];
			// joint_move.call(jp_wam);

			// while(getchar() != '\n');
			// for (size_t i = 0; i < 7; i++)
			// 	jp_wam.request.joints[i] = p4[i];
			// joint_move.call(jp_wam);

			jtm = wamMove(p1,moveTime);
			joint_trajectory_velocity_move_bt.call(jtm);
			jtm = wamMove(p2,moveTime);
			joint_trajectory_velocity_move_bt.call(jtm);
			jtm = wamMove(p3,moveTime);
			joint_trajectory_velocity_move_bt.call(jtm);
			jtm = wamMove(p4,moveTime);
			joint_trajectory_velocity_move_bt.call(jtm);
			jtm = wamMove(p1,moveTime);
			joint_trajectory_velocity_move_bt.call(jtm);

		}

		if (user_input == 'F')
		{
			// Create waypoint matrix
			int size_wp;
			std::cout << "Enter number of waypoints: ";
			std::cin >> size_wp;

			float trajWaypoints[size_wp][7];

			// float **trajWaypoints = NULL;
			// trajWaypoints = new float*[size_wp];
			// for(int x=0; x<10; x++)
			// 	trajWaypoints[x] = new float[7];

			// Put in idle mode
			while(getchar() != '\n');
			std::cout << "[Enter] to idle\n";
		    while(getchar() != '\n');
		    wam_msgs::Hold holdRequest;
			holdRequest.request.hold = false;
			hold_joint_pos.call(holdRequest);

			while(getchar() != '\n');
			for(int x=0; x<size_wp; x++)
			{
				std::cout << "[Enter] to record position " << x << "\n";
		    	while(getchar() != '\n');
				showCurrentPos();
				for (size_t i = 0; i < 7; i++)
				{
					trajWaypoints[x][i] = currJS[i];
				}
			}

			// Display waypoints
			for(int x=0;x<size_wp;x++)
			{
				std::cout << "Waypoint " << x << "\n";
				for (size_t i = 0; i < 7; i++)
				{
					std::cout << trajWaypoints[x][i];
					if (i < 6) std::cout << ", ";                        
				}
				std::cout <<"\n\n";
			}

			// Move WAM
			while(getchar() != '\n');
			std::cout << "[Enter] to move WAM\n";
		    while(getchar() != '\n');

			float next_wp[7];
			moveTime = 6;
			for(int x=0;x<size_wp;x++)
			{
				for(int i=0;i<7;i++)
				{
					next_wp[i] = trajWaypoints[x][i];
				}
				jtm = wamMove(next_wp,moveTime);
				joint_trajectory_velocity_move_bt.call(jtm);
			}
		}

		if (user_input == 'R')
		{
			float initialPos[7];
			float finalPos[7];

			std::cout << "[Enter] to idle\n";
		    while(getchar() != '\n');
		    wam_msgs::Hold holdRequest;
			holdRequest.request.hold = false;
			hold_joint_pos.call(holdRequest);

			// Record initial position
			std::cout << "[Enter] to record initialPos\n";
		    while(getchar() != '\n');
		    ros::spinOnce();
			for (size_t i = 0; i < 7; i++)
			{
				initialPos[i] = currJS[i];
				std::cout << currJS[i];
				if (i < 6) std::cout << ", ";
				if (i == 6) std::cout << "\n\n";                        
			}

			// Record final position
			std::cout << "[Enter] to record finalPos\n";
		    while(getchar() != '\n');
		    ros::spinOnce();
			for (size_t i = 0; i < 7; i++)
			{
				finalPos[i] = currJS[i];
				std::cout << currJS[i];
				if (i < 6) std::cout << ", ";
				if (i == 6) std::cout << "\n\n";                        
			}

			// Move WAM
			int numReps;
			std::cout << "Enter number of reps: ";
		    std::cin >> numReps;
		    while(getchar() != '\n');

			std::cout << "[Enter] to move WAM\n";
		    while(getchar() != '\n');
			moveTime = 4;

			for (size_t i = 0; i < numReps; i++)
			{
				std::cout << "Repetition " << i << " of " << numReps << "\n";
				jtm = wamMove(initialPos,moveTime);
				joint_trajectory_velocity_move_bt.call(jtm);
				jtm = wamMove(finalPos,moveTime);
				joint_trajectory_velocity_move_bt.call(jtm);
				showCurrentPos();
			}

			// Display initial and final JP
			std::cout << "Initial Position:\n";
			for (size_t i = 0; i < 7; i++)
			{
				std::cout << initialPos[i];
				if (i < 6) std::cout << ", ";
				if (i == 6) std::cout << "\n\n";                         
			}

			std::cout << "Final Position:\n";
			for (size_t i = 0; i < 7; i++)
			{
				std::cout << finalPos[i];
				if (i < 6) std::cout << ", ";
				if (i == 6) std::cout << "\n\n";                         
			}

		}

		if (user_input == 'P')
		{
			std::cout << "[Enter] to continue\n";
			while(getchar() != '\n');
			moveTime = 4;
			float pos1[] = {-0.401392, -1.32664, 1.72719, 1.27261, 0.390533, -0.0282284, -0.140658};
			float pos2[] = {-0.401465, -1.29832, 1.68117, 1.21704, 0.44865, 0.0294145, -0.15124};
			float pos3[] = {-0.414686, -1.2738, 1.64035, 1.16744, 0.458692, 0.0416705, -0.152473};
			float pos4[] = {-0.430318, -1.24445, 1.59141, 1.09569, 0.519735, 0.102713, -0.181756};
			float pos5[] = {-0.42305, -1.24437, 1.59136, 1.20298, 0.547805, 0.0553498, -0.195626};
			float pos6[] = {-0.414029, -1.23815, 1.58092, 1.30474, 0.533572, 0.0398519, -0.191208};
			float pos7[] = {-0.418667, -1.22968, 1.56668, 1.38143, 0.532544, 0.0408798, -0.216175};
			float pos8[] = {-0.408769, -1.25344, 1.60641, 1.42797, 0.531121, 0.0392984, -0.234567};
			float pos9[] = {-0.406395, -1.27671, 1.6455, 1.45166, 0.531754, 0.0400891, -0.248745};
			float pos10[] = {-0.392736, -1.29973, 1.68418, 1.49299, 0.530488, 0.0464148, -0.245457};
			float pos11[] = {-0.385102, -1.30323, 1.69007, 1.41101, 0.550652, 0.08002, -0.235594};
			float pos12[] = {-0.389375, -1.3114, 1.70379, 1.32093, 0.553498, 0.0838154, -0.237135};

			jtm = wamMove(pos1,moveTime);
			joint_trajectory_velocity_move_bt.call(jtm);
			showCurrentPos();
			jtm = wamMove(pos2,moveTime);
			joint_trajectory_velocity_move_bt.call(jtm);
			showCurrentPos();
			jtm = wamMove(pos3,moveTime);
			joint_trajectory_velocity_move_bt.call(jtm);
			showCurrentPos();
			jtm = wamMove(pos4,moveTime);
			joint_trajectory_velocity_move_bt.call(jtm);
			showCurrentPos();
			jtm = wamMove(pos5,moveTime);
			joint_trajectory_velocity_move_bt.call(jtm);
			showCurrentPos();
			jtm = wamMove(pos6,moveTime);
			joint_trajectory_velocity_move_bt.call(jtm);
			showCurrentPos();
			jtm = wamMove(pos7,moveTime);
			joint_trajectory_velocity_move_bt.call(jtm);
			showCurrentPos();
			jtm = wamMove(pos8,moveTime);
			joint_trajectory_velocity_move_bt.call(jtm);
			showCurrentPos();
			jtm = wamMove(pos9,moveTime);
			joint_trajectory_velocity_move_bt.call(jtm);
			showCurrentPos();
			jtm = wamMove(pos10,moveTime);
			joint_trajectory_velocity_move_bt.call(jtm);
			showCurrentPos();
			jtm = wamMove(pos11,moveTime);
			joint_trajectory_velocity_move_bt.call(jtm);
			showCurrentPos();
			jtm = wamMove(pos12,moveTime);
			joint_trajectory_velocity_move_bt.call(jtm);
			showCurrentPos();
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
			showCurrentPos();
		}
	}
	
	sleep(10.0);
	
	ROS_INFO("I MADE IT TO THE END!!!");
	ros::waitForShutdown();
	//ros::shutdown();  
	return 0;
}