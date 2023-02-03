#include "string"
#include "ros/ros.h"
#include "serial/serial.h"
#include "oroca_ros_tutorials/MsgTutorial.h"
#include "oroca_ros_tutorials/Header.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/Float32.h"
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <sstream>

/* serial::Serial _serial;				// serial object           /////////////////////////////////////////////////////

void angle_write_callback(const sensor_msgs::JointState & msg) /////////////////////////////////////////////////////
{
	std::string Gcode = "";
	std_msgs::String result;
	char angle0[10];
	char angle1[10];
	char angle2[10];
	char angle3[10];
	char angle4[10];
	char angle5[10];

	sprintf(angle0, "%.2f", msg.position[0]*57.296);
	sprintf(angle1, "%.2f", msg.position[1]*57.296);
	sprintf(angle2, "%.2f", msg.position[2]*57.296);
	sprintf(angle3, "%.2f", msg.position[3]*57.296);
    sprintf(angle4, "%.2f", msg.position[4]*57.296);
	sprintf(angle5, "%.2f", msg.position[5]*57.296);
	Gcode = (std::string)"M50 G0 X" + angle0 + " Y" + angle1 + " Z" + angle2 + " A" + angle3 + "B" + angle4 + "C" + angle5 + " F3000" + "\r\n";
	ROS_INFO("%s", Gcode.c_str());
	_serial.write(Gcode.c_str());
	result.data = _serial.read(_serial.available());
	
	//ROS_INFO("seq: %d", msg->seq);   // 수신된 메시지를 표시하는 함수
} */

int main(int argc, char **argv)
{
    ros::init(argc, argv, "topic_pub2");
    ros::NodeHandle nh;
    ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 100);
    //ros::Subscriber ros_tutorial_sub = nh.subscribe("/joint_states", 100, angle_write_callback); //////////////////////////////////////////
    tf::TransformBroadcaster broadcaster;
    ros::Rate loop_rate(20);

    const double degree = M_PI/180;
    const double indegree = -1 * M_PI/180;

    //int count = 0;

    // robot state
    double joint1=0, joint2=0, joint3=0, joint4=0, joint5=0, joint6=0, angle=0, tilt=degree, intilt=indegree;

    //message declarations
    geometry_msgs::TransformStamped trans;
    sensor_msgs::JointState joint_states;
    trans.header.frame_id = "Link1";
    trans.child_frame_id = "Link2";

    std::string robot_description_string;
    nh.param("robot_description",robot_description_string, std::string());

    std::vector<std::string> joint_name = {"joint1","joint2","joint3","joint4","joint5","joint6"}; //각 joint의 이름
    std::vector<double> joint = {joint1,joint2,joint3,joint4,joint5,joint6}; //각 joint의 position 값


    //oroca_ros_tutorials::MsgTutorial topic_msg;

    /* try//로봇팔의 직렬 연결 시도                                                //////////////////////////////////////////////////////////
	{
		_serial.setPort("/dev/ttyUSB0");
		_serial.setBaudrate(115200);
		serial::Timeout to = serial::Timeout::simpleTimeout(1000);
		_serial.setTimeout(to);
		_serial.open();
		_serial.write("M50\r\n");
		ROS_INFO_STREAM("Port has been open successfully");
	}
	catch (serial::IOException& e)
	{
		ROS_ERROR_STREAM("Unable to open port");
		return -1;
	}
	
	if (_serial.isOpen())
	{
		ros::Duration(1).sleep();				
		ROS_INFO_STREAM("Attach and wait for commands");
	}                                                                         /////////////////////////////////////////////////////////
     */


    while (ros::ok())
    {
        //update joint_state
        joint_states.header.stamp = ros::Time::now();
        joint_states.name.resize(6);
        joint_states.position.resize(6);
        joint_states.name[0] = "joint1";
        joint_states.position[0] = joint1;
        joint_states.name[1] = "joint2";
        joint_states.position[1] = joint2;
        joint_states.name[2] = "joint3";
        joint_states.position[2] = joint3;
        joint_states.name[3] = "joint4";
        joint_states.position[3] = joint4;
        joint_states.name[4] = "joint5";
        joint_states.position[4] = joint5;
        joint_states.name[5] = "joint6";
        joint_states.position[5] = joint6;


        // update transform
        trans.header.stamp = ros::Time::now();
        trans.transform.translation.x=cos(angle)*2;
        trans.transform.translation.y=sin(angle)*2;
        trans.transform.translation.z=.7;
        trans.transform.rotation=tf::createQuaternionMsgFromYaw(angle+M_PI/2);


        // send the joint state and transform
        joint_pub.publish(joint_states);
        broadcaster.sendTransform(trans);


        //Create new robot state
        if(joint1<=1. && joint1>=0) joint1+=tilt;
        if(joint2<=2 && joint2>=0) joint2+=tilt;
        if(joint3>=-2. && joint3<=0) joint3+=intilt;

        if(joint1>1.&&joint2>2&&joint3<-2)
        {
            tilt *= -1;
            intilt *= -1; joint1+=tilt; joint2+=tilt; joint3+=intilt;
        }

        
        /* joint1 += tilt;
        if (joint1<-.5 || joint1>0) tilt *= -1; */
        /* if(joint1<=1. && joint1>=0) joint1+=tilt;
        if(joint2<=2 && joint2>=0) joint2+=tilt;
        if(joint3>=-2. && joint3<=0) joint3+=intilt; */
        /* if(joint1 != 0) joint1 -=tilt;
        if(joint2 != 0) joint2 -=tilt;
        if(joint3 != 0) joint3 -= intilt; */



        /* for (int k=0; k<6; k++)
        {
            /* topic_msg.name[k] = joint_name[k];
            topic_msg.position[k]=joint[k]; */
        //}
        /* ROS_INFO("sec = %d", topic_msg.stamp.sec);
        ROS_INFO("nsec = %d", topic_msg.stamp.nsec);
        ROS_INFO("seq = %d", topic_msg.seq);
        ros_tutorial_pub.publish(topic_msg); */

        ros::spinOnce();                                                                        ////////////////////////////////////////////////////
        loop_rate.sleep();
        
    }
    return 0;
}