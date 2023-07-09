#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <open_manipulator_msgs/SetJointPosition.h>

ros::Publisher pub_theta;
ros::Subscriber sub_theta;

std_msgs::Float32MultiArray theta_msg;
std::vector<std::string> joint_name_global;
std::vector<double> joint_angle_global;

double joint_1, joint_2, joint_3, joint_4;

// ros::ServiceClient goal_joint_space_path_client_;
  // goal_joint_space_path_client_ = n.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_joint_space_path");

// open_manipulator_msgs::SetJointPosition srv;

bool setJointSpacePath(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time);
void sensorCallback(const std_msgs::Float32MultiArray& msg);




int main(int argc, char** argv)
{
    ros::init(argc, argv, "input_node_controller");
    ros::NodeHandle nh;
    // ros::Rate rate(500);
    sub_theta = nh.subscribe("pos_robot", 10, sensorCallback);
    // bool topic_availabe = ros::topic::waitForMessage<std_msgs::String>("pos_robot", ros::Duration(5.0));
    std::cout<<"test test test"<<std::endl;
    // open_manipulator_msgs::SetJointPosition srv;



    std::cout << joint_1 << " " << joint_2 << " " << joint_3 << " " << joint_4 << std::endl;

    // while(ros::ok())
    // {
    // ros::Duration delay_duration(2.0);
    // ROS_INFO("TUNGGU SEK 2 DETIK...");

    // delay_duration.sleep();
    
    // std::cout << joint_1 << " " << joint_2 << " " << joint_3 << " " << joint_4 << std::endl;



    // double path_time = 4.0;
    // joint_name_global.push_back("joint1"); 
    // joint_name_global.push_back("joint2"); 
    // joint_name_global.push_back("joint3"); 
    // joint_name_global.push_back("joint4"); 

    // joint_angle_global.push_back(joint_1);
    // joint_angle_global.push_back(joint_2);
    // joint_angle_global.push_back(joint_3);
    // joint_angle_global.push_back(joint_4);

    // srv.request.joint_position.joint_name = joint_name_global;
    // srv.request.joint_position.position = joint_angle_global;
    // srv.request.path_time = path_time;

    std::cout << joint_1 << " " << joint_2 << " " << joint_3 << " " << joint_4 << std::endl;


    // goal_joint_space_path_client_.call(srv);
    // {
      ROS_INFO("OKE BAGUS LANJUTKAN");
      // return srv.response.is_planned;
    // }
    // else
    // {
      // ROS_ERROR("HILAL TIDAK MUNCUL");
      // return 1;
    // }

    

    // setJointSpacePath(joint_name_global, joint_angle_global, path_time);

        // std::cout << joint_1 << " " << joint_2 << " " << joint_3 << " " << joint_4 << std::endl;

        // pub_theta = nh.advertise<std_msgs::Float32MultiArray>("theta", 1000);
        std::cout<<"test3"<<std::endl;
        

        // ROS_WARN("HILAL BELUM TERLIHAT");
        

        // ros::spinOnce();
        // rate.sleep();

    // }

    ros::spin();
    return 0;
}

bool setJointSpacePath(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time)
{
//   open_manipulator_msgs::SetJointPosition srv;
  // srv.request.joint_position.joint_name = joint_name;
  // srv.request.joint_position.position = joint_angle;
  // srv.request.path_time = path_time;
  

  // std::cout<<"test1"<<std::endl;

  // if(goal_joint_space_path_client_.call(srv))
  // {
  //   ROS_INFO("OKE BAGUS LANJUTKAN");
  //   return srv.response.is_planned;
  // }
  //   std::cout<<"test4"<<std::endl;

  // return false;
}

void sensorCallback(const std_msgs::Float32MultiArray& msg)
{
    ros::NodeHandle nh;
    ros::ServiceClient goal_joint_space_path_client_ = nh.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_joint_space_path");
    open_manipulator_msgs::SetJointPosition srv;

    theta_msg.data = msg.data;

    joint_1 = theta_msg.data[0];
    joint_2 = theta_msg.data[1];
    joint_3 = theta_msg.data[2];
    joint_4 = theta_msg.data[3];

    double path_time = 4.0;
    joint_name_global.push_back("joint1"); 
    joint_name_global.push_back("joint2"); 
    joint_name_global.push_back("joint3"); 
    joint_name_global.push_back("joint4"); 

    joint_angle_global.push_back(0);
    joint_angle_global.push_back(0);
    joint_angle_global.push_back(0);
    joint_angle_global.push_back(0);
      
    std::cout << theta_msg << std::endl;
    
    std::cout<<"test2"<<std::endl;

    srv.request.joint_position.joint_name = joint_name_global;
    srv.request.joint_position.position = joint_angle_global;
    srv.request.path_time = path_time;

    std::cout<<srv.request<<std::endl;

    // goal_joint_space_path_client_.call(srv);

    std::cout<<"test sampai sini"<<std::endl;

    if(goal_joint_space_path_client_.call(srv))
    {
      ROS_INFO("OKE BAGUS LANJUTKAN");
      // return srv.response.is_planned;
    }
    else
    {
      ROS_ERROR("HILAL TIDAK MUNCUL");
      // return 1;
    }

    joint_name_global.clear();
    joint_angle_global.clear();

    std::cout<<"test sampai situ"<<std::endl;

    
    // setJointSpacePath(joint_name_global, joint_angle_global, path_time);



}