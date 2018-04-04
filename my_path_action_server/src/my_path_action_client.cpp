#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <my_path_action_server/path_msgAction.h>
#include <example_ros_service/PathSrv.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <string>
#include <math.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h> 
using namespace std;

bool g_lidar_alarm=false; // global var for lidar alarm
bool g_finished = false; //global var for completed goal
geometry_msgs::Pose g_current_pose;

geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(phi / 2.0);
    quaternion.w = cos(phi / 2.0);
    return quaternion;
}

double convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion) {
    double quat_z = quaternion.z;
    double quat_w = quaternion.w;
    double phi = 2.0 * atan2(quat_z, quat_w); // cheap conversion from quaternion to heading for planar motion
    return phi;
}

void alarmCallback(const std_msgs::Bool& alarm_msg) 
{ 
    ROS_INFO("lidar_alarm callback");
    g_lidar_alarm = alarm_msg.data; //make the alarm status global, so main() can use it
    if (g_lidar_alarm) {
       ROS_INFO("LIDAR alarm received!"); 
    }
} 

void feedbackCb(const my_path_action_server::path_msgFeedbackConstPtr& fdbk_msg) {
    g_current_pose = fdbk_msg->cur_pos; //make status available to "main()"
}

void doneCb(const actionlib::SimpleClientGoalState& state, const my_path_action_server::path_msgResultConstPtr& result){
        g_finished = true;
        ROS_INFO("doneCB: this is the result message!");
        ROS_INFO("my_path_action_server executed your request for pose!");
}

int main(int argc, char** argv) {
    ros::init(argc,argv, "my_path_action_client");
    actionlib::SimpleActionClient<my_path_action_server::path_msgAction> action_client("path_action", true);
    my_path_action_server::path_msgGoal goal;

    // attempt to connect to the server:
    ROS_INFO("waiting for server: ");
    bool server_exists = action_client.waitForServer(ros::Duration(20.0)); // wait for up to 20 seconds
   
    if (!server_exists) {
        ROS_WARN("could not connect to server; halting");
        return 0; 
    }

    ROS_INFO("connected to action server");  
    while(true){
       // ros::Rate timer(2.0);
        ros::NodeHandle n;
        ros::Subscriber alarm_subscriber = n.subscribe("lidar_alarm",1,alarmCallback);  //will check for lidar_alarm
        nav_msgs::Path navigation_path;     
  
        const double sample_dt = 0.01; //specify a sample period of 10ms  
        const double speed = 0.5; // .5 m/s speed command
        const double yaw_rate = M_PI/4; //Pi/4 rad/sec yaw rate command
        const double time_half_sec = 0.5; //.5 seconds
        const double time_1_sec = 1.0; //1 second
        const double time_3_sec = 3.0; //3 seconds
        geometry_msgs::Quaternion quat;
        double srt_two = sqrt(2);  //Define square root of 2
        double PI = acos(-1);   //Define Pi

        geometry_msgs::PoseStamped pose_stamped;
        geometry_msgs::Pose pose;
        
        bool lastMsg;
        lastMsg = ros::topic::waitForMessage <std_msgs::Bool>("lidar_alarm");  //ensures lidar_alarm is running
        ROS_INFO("message is %d", lastMsg);
        if(!g_lidar_alarm){
           //action_client.cancelAllGoals();
           ROS_INFO("No alarm");
           //Pose 1
           pose.position.x = 3.0; // say desired x-coord is 3
           pose.position.y = 0.0;
           pose.position.z = 0.0; // let's hope so!
           quat = convertPlanarPhi2Quaternion(PI/4);
           pose_stamped.pose.orientation = quat;
           pose_stamped.pose = pose;
           navigation_path.poses.push_back(pose_stamped);
    
           //Pose 2
           pose_stamped.pose.position.x = 6.5;
           pose_stamped.pose.position.y = 3.5;
           quat = convertPlanarPhi2Quaternion(PI/2);
           pose_stamped.pose.orientation = quat;
           navigation_path.poses.push_back(pose_stamped);

           //Pose 3
           pose_stamped.pose.position.x = 6.5;
           pose_stamped.pose.position.y = 11;
           quat = convertPlanarPhi2Quaternion(PI);
           pose_stamped.pose.orientation = quat;
           navigation_path.poses.push_back(pose_stamped);

           //Pose 5
           pose_stamped.pose.position.x = 3;
           pose_stamped.pose.position.y = 11;
           quat = convertPlanarPhi2Quaternion(PI/2);
           pose_stamped.pose.orientation = quat;
           navigation_path.poses.push_back(pose_stamped);

           //Pose 6
           pose_stamped.pose.position.x = 3;
           pose_stamped.pose.position.y = 12;
           quat = convertPlanarPhi2Quaternion(PI);
           pose_stamped.pose.orientation = quat;
           navigation_path.poses.push_back(pose_stamped);

           //Pose 7
           pose_stamped.pose.position.x = 0;
           pose_stamped.pose.position.y = 12;
           quat = convertPlanarPhi2Quaternion(3*PI/4);
           pose_stamped.pose.orientation = quat;
           navigation_path.poses.push_back(pose_stamped);

           goal.nav_path = navigation_path;
           action_client.sendGoal(goal,&doneCb);
           ROS_INFO("Sent Pose");
        }
        
       //action_client.isPreemptRequested(); // THIS LINE NEEDS TO CANCEL CURRENT GOAL
        //SUPPOSED TO SPIN HERE
           //ros::Publisher twist_cmd_publisher = n.advertise<geometry_msgs::Twist>("/robot0/cmd_vel", 1);
       if(g_lidar_alarm){
           action_client.cancelAllGoals();
           ROS_INFO("ALARM ON");
           //set position to
           double new_ang = convertPlanarQuat2Phi(g_current_pose.orientation) + PI/4; //Will rotate 45 degrees
           pose_stamped.pose.orientation = convertPlanarPhi2Quaternion(new_ang);
           ROS_INFO("Will Rotate 45 degrees");  
           pose_stamped.pose.position = g_current_pose.position;
           navigation_path.poses.push_back(pose_stamped);
           goal.nav_path = navigation_path;         
           action_client.sendGoal(goal,&doneCb);
       }
       

       bool finished_before_timeout = action_client.waitForResult(ros::Duration(60.0));

       if(!finished_before_timeout) {
          ROS_INFO("Did not finish in 60 seconds");
          return 0;
        } 
        //timer.sleep();
        ros::spinOnce();
    }
    return 0;
}

