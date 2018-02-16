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

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

//Kinematic info
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/RobotState.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <trajectory_msgs/JointTrajectory.h>

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

//JSON library to manipulate UDP packets
//#include "rapidjson/document.h"     // rapidjson's DOM-style API
//#include "rapidjson/prettywriter.h" // for stringify JSON
//#include "rapidjson/filestream.h"   // wrapper of C stream for prettywriter as output
#include <boost/thread.hpp>
#include <boost/function.hpp>
#include <signal.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <netdb.h>

//Matlab
#define serverPORT "55000" // the port client will be connecting to
#define MAXDATASIZE 200 // max number of bytes we can get at once
#define BACKLOG 10   // how many pending connections queue will hold

bool perception = true;
bool trajlive = true;

sensor_msgs::JointState currJointState;
void wamJointStateCallback(const sensor_msgs::JointState::ConstPtr& jointState)
{
    /*ROS_INFO("IN wamJointStateCallback");
    ROS_INFO("J[%4.2f, %4.2f, %4.2f, %4.2f, %4.2f, %4.2f, %4.2f]", jointState->position[0],jointState->position[1],
    jointState->position[2],jointState->position[3],jointState->position[4],jointState->position[5],jointState->position[6]); */
    currJointState = *jointState;
}
    
std::string actionSpace("na");
std::string saveFilename("na");
std::string episode("na");

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wam_digging");
    ros::NodeHandle nh("bhand"); 
    ros::NodeHandle nw("wam");
    ros::AsyncSpinner spinner(1);
    spinner.start();
  
    sleep(0.01);

    // BEGIN_TUTORIAL
    // 
    // Setup
    // ^^^^^
    // 
    // The :move_group_interface:`MoveGroup` class can be easily 
    // setup using just the name
    // of the group you would like to control and plan for.
    /*
    moveit::planning_interface::MoveGroup group("right_wam");
    const robot_model::RobotModelConstPtr kinematic_model = group.getCurrentState()->getRobotModel();
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
	
	
    const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("right_wam");

    kinematic_state->setToDefaultValues(joint_model_group, "right_wam_dig");
    const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();

    std::vector<double> joint_values;
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    for(std::size_t i = 0; i < joint_names.size(); ++i)
        ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    
    // We will use the :planning_scene_interface:`PlanningSceneInterface`
    // class to deal directly with the world.
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // (Optional) Create a publisher for visualizing plans in Rviz.
    ros::Publisher display_publisher = nw.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;
    */

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

    //wam/ros/kinetic/setup.bash
    ros::ServiceClient joint_move  = nw.serviceClient<wam_msgs::JointMove>("joint_move");
    ros::ServiceClient go_home = nw.serviceClient<std_srvs::Empty>("go_home");
    ros::ServiceClient joint_trajectory_velocity_move  = nw.serviceClient<wam_msgs::JointTrajectoryVelocityMove>("joint_trajectory_velocity_move");
    ros::ServiceClient joint_trajectory_velocity_move_bt  = nw.serviceClient<wam_msgs::JointTrajectoryVelocityMove>("joint_trajectory_velocity_move_bt");
    ros::ServiceClient biotac_logger = nw.serviceClient<wam_msgs::LoggerInfo>("logger_controller");
    ros::ServiceClient special_service = nw.serviceClient<wam_msgs::SpecialService>("special_service");
    ros::ServiceClient hold_joint_pos = nw.serviceClient<wam_msgs::Hold>("hold_joint_pos"); //change made by Nancy

    std_srvs::Empty empty_srv_1, empty_srv_2;
    wam_msgs::JointTrajectoryVelocityMove jointTrajectoryMove;
    wam_msgs::LoggerInfo loggerController;
    wam_msgs::SpecialService specialService;
    //wam_msgs::Hold holdjointposition; //change made by Nancy

    //Place hand in safe grasp position
    close_spread.call(empty_srv_1);
    if(open_grasp.call(empty_srv_1))
        ROS_INFO("CLOSE_GRASP success");
    else
        ROS_WARN("CLOSE_GRASP fail");
    sleep(2.0);

    std::vector<wam_msgs::JointMove> moveToStartingPoint;
    wam_msgs::JointMove jointMovePos;
    wam_msgs::Hold idle; 
	
    
    for (int i=0; i<4; i++)
        moveToStartingPoint.push_back(jointMovePos);
    //Move to new starting point
    float pos0[] = {-1.1271471689271135, -1.632264158720084, 0.172413766412171, -0.00681769239060285, -0.1468351712940018, -1.4769704709750462, 0.09832683282026516};
    moveToStartingPoint[0].request.joints.assign(pos0, pos0+7);
    float pos1[] = {-1.054465698263008, -0.7165726537108978, -0.3992427770914637, -0.672224469713441, -0.12390453065035056, -1.4984778304752984, 0.10284760674303597};
    moveToStartingPoint[1].request.joints.assign(pos1, pos1+7);
    float pos2[] = {-1.625289168116929, -0.6005059657779577, 0.13273123287286187, -0.5965480841777493, -0.14398860900720378, -1.4617888054454562, 0.0875386223227439};
    moveToStartingPoint[2].request.joints.assign(pos2, pos2+7);
    float pos3[] = {-1.6122503314199013, -1.0651528189438844, 0.13446449295848684, -0.5893895071676163, -0.15252829586759797, -1.4529328338865288, 0.08723038773710044};
    moveToStartingPoint[3].request.joints.assign(pos3, pos3+7);

    wam_msgs::BHandSpreadPos spreadPos;
    wam_msgs::BHandFingerPos fingerPos;

    while(getchar() != '\n');
/*
    fingerPos.request.radians[0] = 0; fingerPos.request.radians[1] = 0; fingerPos.request.radians[2] = 0;
    
    finger_pos.call(fingerPos);
    close_spread.call(empty_srv_1);
    usleep(1000000);
*/

    idle.request.hold = false;
    if(hold_joint_pos.call(idle))
    {
        ROS_INFO("IDLE SUCCESS***************");
    }
    else
    {
        ROS_WARN("joint_move.call FAIL***************");
    }
    //Move to starting point
    /*
    for(int i=0; i<moveToStartingPoint.size(); i++)
    {

        
        if(joint_move.call(moveToStartingPoint[i]))
        {
            ROS_INFO("joint_move.call SUCCESS***************");
            while(getchar() != '\n');
        }
        else
            ROS_WARN("joint_move.call FAIL***************");
            
    }
    */

/*
    loggerController.request.filename = "startUp";
    loggerController.request.record = 1;
    if(biotac_logger.call(loggerController))
      ROS_INFO("1s-biotac_logger success status = %ld",loggerController.response.success);
    else
      ROS_INFO("1s-biotac_logger failed");
    usleep(1000000);


    ROS_INFO("STARTING MOVEIT...");
    ROS_INFO_NAMED("tutorial", "End effector link: %s", group.getEndEffectorLink().c_str());
    // Group Specific
    group.setPlannerId("RRTstarkConfigDefault");
    //group.setPlannerId("ESTkConfigDefault");

    // 1st Planned Movement
    std::vector<double> start_variable_values;
    group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), start_variable_values);
    //Starting joint space for the planner. It's important to provide joint space since infinte solutions to cartesian position
    //double startPoint[] = {-1.52448,-1.5671, 0.400839, 0.563994, -0.108723, 1.52046, 0.39043}; // old low
    //double startPoint[] = {-2.01872, -1.00426, 0.957535, -0.851274, 0.754497, 1.5286, 0.0132541}; // new low
    //double startPoint[] = {-0.928, -1.137,  2.118,  0.705, -1.109, -1.834, -2.521}; // digging
    //double startPoint[] = {0.5, 0,  0,0,0,0,0}; // test
    //double startPoint[] = {-0.583935, -1.29061, 0.566958, 1.7807, 0.469841, 0.953598, -1.32551}; // old high
    //double startPoint[] = {-1.46309,-0.90437,1.28343,0.871301,0.244014,0.900146,2.16822}; // orig kenny
    //start_variable_values.assign(startPoint, startPoint+7);
    group.setStartState(*kinematic_state);
    //kinematic_state->setJointGroupPositions(joint_model_group, start_variable_values);
    //group.setStartState(*kinematic_state);

    geometry_msgs::Pose start_pose;
    setGeometryPoseFromInitialKinematicState(start_pose, *kinematic_state);

    double delta_z;

    // Set absolute orientation of hand (quaternion)
    //start_pose.orientation.w = 0.5;
    //start_pose.orientation.x = 0.5;
    //start_pose.orientation.y = 0.5;
    //start_pose.orientation.z = -0.5;
    //start_pose.orientation.w = 1;
    //.position.x += -0.01;
    //start_pose.position.y += 0;
    //start_pose.position.z += 0.01;

    start_pose.orientation.w = 0;
    start_pose.orientation.x = 1;
    start_pose.orientation.y = 0;
    start_pose.orientation.z = 0;
    start_pose.position.x += +0.3;
    start_pose.position.y += 0;
    start_pose.position.z += 0;
    double x_pos = start_pose.position.x;
    double y_pos = start_pose.position.y;
    double z_pos = start_pose.position.z;
    
    moveit::planning_interface::MoveGroup::Plan my_plan;
    //group.setOrientationTarget(start_pose.orientation.x, start_pose.orientation.y, start_pose.orientation.z, start_pose.orientation.w);
    //group.setPoseTarget(start_pose); //Cartesian position and orientation goal
    group.setPositionTarget(start_pose.position.x, start_pose.position.y, start_pose.position.z); //Cartesian position and orientation goal
    ROS_INFO("~~~~~~~~~~~~~~~~~");
    group.setPlanningTime(5);
    bool success = group.plan(my_plan); //Start planner

    bool operate_WAM = false;
    ROS_INFO("~~~~~~~~~~~~~~~~~");
    sleep (5.0);
    
    moveit_msgs::RobotTrajectory current_Traj = my_plan.trajectory_;
    trajectory_msgs::JointTrajectory jointTrajectory = current_Traj.joint_trajectory;
    
    std::vector<double> trajectroyStartPoint;
    int itToLastPoint = jointTrajectory.points.size()-1;
    trajectroyStartPoint = jointTrajectory.points[itToLastPoint].positions;
    moveit::planning_interface::MoveGroup::Plan my_plan1;

    trajectroyStartPoint.clear();
    trajectroyStartPoint = jointTrajectory.points[jointTrajectory.points.size()-1].positions;
    kinematic_state->setJointGroupPositions(joint_model_group, trajectroyStartPoint);
    
    geometry_msgs::Pose curr_pose;
    curr_pose = start_pose;

    curr_pose.position.z -= .10;

    group.setStartState(*kinematic_state);
    group.setPoseTarget(curr_pose); //Cartesian position and orientation goal

    group.setPlanningTime(3);
    success = group.plan(my_plan1);
    */
    /*
    // Excuting Trajectory Planned on the WAM
    
    if(operate_WAM)
    {
        //double travelLength(0);
        //travelLength = getEndEffectorTravel( group, jointTrajectory); 

        jointTrajectoryMove.request.jointTrajectory = jointTrajectory;
        //ROS_INFO("travelLength of orientation move - %6.4f", travelLength);
        ROS_INFO("SHALL WE BEGIN????");
        // Used in Trajectory Spline
        jointTrajectoryMove.request.length = 0.15; //travel length of end effector
        jointTrajectoryMove.request.velocity = 0.05; // 0.02 m/s or 2 cm/s
        
        if(joint_trajectory_velocity_move.call(jointTrajectoryMove))
            ROS_INFO("JointTrajectoryVelocityMove success status = %lu",jointTrajectoryMove.response.status);
        else
            ROS_WARN("JointTrajectoryVelocityMove FAIL ######");

        group.setPlanningTime(3);
        success = group.plan(my_plan);
    }
    */


    // 2nd movement
    /*
    while(true)
    {
        
        char traj;
        double delta_z;
        
        cout << string( 100, '\n' );
        std::cout << "Enter desired trajectory: ";
        std::cin >> traj;

        // 5N Force
        if(traj == 'A') delta_z = -0.02;
        if(traj == 'B') delta_z = -0.04; 
        //if(traj == 'C') delta_z = -0.06; 
        //if(traj == 'D') delta_z = -0.08; 
        //if(traj == 'E') delta_z = -0.10;
        if(traj == 'X') break; 
        
        delta_z = -.10;
        
        // MOVE DOWN FOR FORCE MEASUREMENT
        //std::vector<double> trajectroyStartPoint;
        //int itToLastPoint = jointTrajectory.points.size()-1;
        //trajectroyStartPoint = jointTrajectory.points[itToLastPoint].positions;
        moveit::planning_interface::MoveGroup::Plan my_plan1; // Plan for 2nd Position

        geometry_msgs::Pose curr_pose;
        curr_pose = start_pose;

        curr_pose.position.x -= delta_z;
        //setGeometryPoseFromInitialKinematicState(curr_pose, *curr_state);

        //trajectroyStartPoint.clear();
        //trajectroyStartPoint = jointTrajectory.points[jointTrajectory.points.size()-1].positions;
        //kinematic_state->setJointGroupPositions(joint_model_group, trajectroyStartPoint);
        group.setStartState(*kinematic_state);
        group.setPoseTarget(curr_pose); //Cartesian position and orientation goal

        group.setPlanningTime(3);
        bool success = group.plan(my_plan1);

        moveit_msgs::RobotTrajectory current_Traj = my_plan1.trajectory_;
        trajectory_msgs::JointTrajectory jointTrajectory = current_Traj.joint_trajectory;
        
        if(operate_WAM)
        {
            if(!success)
                while(true) ROS_INFO("YOU FAILED....");
                
            if(success)
            {
                current_Traj = my_plan1.trajectory_;
                jointTrajectory = current_Traj.joint_trajectory;

                jointTrajectoryMove.request.jointTrajectory = jointTrajectory;
                jointTrajectoryMove.request.length = delta_z;
                jointTrajectoryMove.request.velocity = 0.03;

                if(joint_trajectory_velocity_move.call(jointTrajectoryMove))
                {
                    while(true) ROS_INFO("3....");
                    ROS_INFO("JointTrajectoryVelocityMove success status = %lu",jointTrajectoryMove.response.status);
                }
                else
                {
                    while(true) ROS_INFO("4....");
                    ROS_WARN("JointTrajectoryVelocityMove FAIL ######");
                }
            }
        }
        

        // MOVE BACK TO STARTING POSITION
        moveit::planning_interface::MoveGroup::Plan my_plan2; // Plan for 2nd Position
        robot_state::RobotState curr_state(*kinematic_state);
        const robot_state::JointModelGroup *joint_model_group_1 = curr_state.getJointModelGroup(group.getName());
        curr_state.setFromIK(joint_model_group_1, curr_pose);
        group.setStartState(curr_state);
        group.setPoseTarget(start_pose); //Cartesian position and orientation goal

        group.setPlanningTime(3);
        success = group.plan(my_plan2);   

        if(operate_WAM)
        {
            if(!success)
                ROS_INFO("YOU FAILED....");

            if(success)
            {
                current_Traj = my_plan2.trajectory_;
                jointTrajectory = current_Traj.joint_trajectory;

                jointTrajectoryMove.request.jointTrajectory = jointTrajectory;
                jointTrajectoryMove.request.length = delta_z;
                jointTrajectoryMove.request.velocity = 0.03;

                if(joint_trajectory_velocity_move.call(jstart_pose.position.zointTrajectoryMove))
                    ROS_INFO("JointTrajectoryVelocityMove success status = %lu",jointTrajectoryMove.response.status);
                else
                    ROS_WARN("JointTrajectoryVelocityMove FAIL ######");
            }
        }
        
        sleep(10.0);
    }
    */
    /*
    // Prepare hand to return to home. Should have if/else for failed ROS srv calls
    fingerPos.request.radians[0] = 0; fingerPos.request.radians[1] = 0;
    finger_pos.call(fingerPos);
    usleep(1000000);
    close_spread.call(empty_srv_1);
  */
    //Reverse to from starting point vector
    /*
    if(operate_WAM)
    {
        for(int i= moveToStartingPoint.size()-2; i>=0; i--)
        {
            joint_move.call(moveToStartingPoint[i]);
        }

        go_home.call(empty_srv_1);
        fingerPos.request.radians[2] = 0;
        finger_pos.call(fingerPos);
    }
*/
    
    sleep(1.0);

    ROS_INFO("I MADE IT TO THE END!!!");
    ros::shutdown();  
    return 0;
}

