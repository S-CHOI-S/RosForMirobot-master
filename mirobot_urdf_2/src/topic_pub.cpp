#include "string"
#include "ros/ros.h"
#include "serial/serial.h"
#include "oroca_ros_tutorials/MsgTutorial.h"
#include "oroca_ros_tutorials/Header.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
//#include <tf/transform_broadcaster.h>
//#include <tf/LinearMath/Quaternion.h>


// void poseCallback(tf::pose& msg)
// {   tf::Quaternion q;
//     tf::TransformBroadcaster broadcaster;
//     tf::Transform transform;
//     transform.setOrigin( tf::Vector3(msg->x, msg->y, msg->z) );
//     q.setRPY(3.1402, -1.5708, 1.5722);
//     transform.setRotation(q);

//     broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0,0,0,1),tf::Vector3(0,0,0.065406)), ros::Time::now(), "base_link","Link1"));
//     //broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(q),tf::Vector3(0.029687, -0.022, 0.061593)), ros::Time::now(), "Link1","Link2"));
//     ROS_INFO("sendTransform");

// }

int main(int argc, char **argv)
{
ros::init(argc, argv, "topic_pub");
ros::NodeHandle nh;
ros::Publisher ros_tutorial_pub = nh.advertise<oroca_ros_tutorials::MsgTutorial>("topic_msg", 100);
//ros::Subscriber sub = nh.subscribe("pose",100,&poseCallback);
//tf::Transform transform;

ros::Rate loop_rate(10);

double joint1=0, joint2=0, joint3=0, joint4=0, joint5=0, joint6=0, angle=0;


//geometry_msgs::TransformStamped odom_trans;
//geometry_msgs::Quaternion quaternion;
oroca_ros_tutorials::MsgTutorial topic_msg;

std::string robot_description_string;
nh.param("robot_description",robot_description_string, std::string());

std::vector<std::string> joint_name = {"joint1","joint2","joint3","joint4","joint5","joint6"}; //각 joint의 이름
std::vector<double> joint = {joint1,joint2,joint3,joint4,joint5,joint6}; //각 joint의 position 값

int count = 0;

/* tf::Quaternion q;
tf::TransformBroadcaster broadcaster;
tf::Transform transform; */

//broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0,0,0,1),tf::Vector3(0,0,0.065406)), ros::Time::now(), "base_link","Link1"));
//broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0,0,0,1),tf::Vector3(0.029687, -0.022, 0.061593)), ros::Time::now(), "Link1","Link2"));


while (ros::ok())
{
    // broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0,0,0,1),tf::Vector3(0,0,0.065406)), ros::Time::now(), "base_link","Link1"));
    // broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0,0,0,1),tf::Vector3(0.029687, -0.022, 0.061593)), ros::Time::now(), "Link1","Link2"));
    //broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0,0,0,1),tf::Vector3(0.029687, -0.022, 0.061593)), ros::Time::now(), "Link2","Link3"));
    //broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0,0,0,1),tf::Vector3(0.029687, -0.022, 0.061593)), ros::Time::now(), "Link3","Link4"));
    //broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0,0,0,1),tf::Vector3(0.029687, -0.022, 0.061593)), ros::Time::now(), "Link4","Link5"));

// // transform.rotation=quaternion.Euler(tf::Vector3(0,0,0.065406));
// // broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "dummy","base_link"));


// // broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "Link2","Link3"));
// // broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "Link3","Link4"));
// // broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "Link4","Link5"));
// // broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "Link5","Link6"));

    count += 0.001;

topic_msg.stamp = ros::Time::now();
topic_msg.seq = count;
topic_msg.name.resize(6); //topic_msg의 name값 4개로 확장
topic_msg.position.resize(6); //topic_msg의 position값 6개로 확장

// odom_trans.header.stamp = ros::Time::now();
// odom_trans.transform.translation.x = cos(angle);
// odom_trans.transform.translation.y = sin(angle);
// odom_trans.transform.translation.z = .7;
// odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(angle+M_PI/2);

//angle+=5;

for (int k=0; k<6; k++)
{
    topic_msg.name[k] = joint_name[k];
    topic_msg.position[k]=joint[k];
}
ROS_INFO("sec = %d", topic_msg.stamp.sec);
ROS_INFO("nsec = %d", topic_msg.stamp.nsec);
ROS_INFO("seq = %d", topic_msg.seq);
ros_tutorial_pub.publish(topic_msg);
//broadcaster.sendTransform(odom_trans);
loop_rate.sleep();
++count;
}
//return 0;
}