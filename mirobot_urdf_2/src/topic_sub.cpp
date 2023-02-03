#include "string"
#include "ros/ros.h"
#include "serial/serial.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/Float32.h"
#include <tf/transform_listener.h>
#include "oroca_ros_tutorials/MsgTutorial.h"
#include <tf/transform_broadcaster.h>
#include <tf/LinearMath/Quaternion.h>



//serial::Serial _serial;				// serial object

void angle_write_callback(const oroca_ros_tutorials::MsgTutorial& msg)
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
	// _serial.write(Gcode.c_str());
	// result.data = _serial.read(_serial.available());
	
	//ROS_INFO("seq: %d", msg->seq);   // 수신된 메시지를 표시하는 함수
}

int main(int argc, char **argv) // Node Main Function
{

	ros::init(argc, argv, "topic_sub"); 
	
	ros::NodeHandle nh;

	ros::Subscriber ros_tutorial_sub = nh.subscribe("/topic_msg", 100, angle_write_callback);
	
	tf::TransformBroadcaster broadcaster;
	tf::Transform transform;
	tf::Quaternion q;
	
	tf::TransformListener listener;

	ros::Rate loop_rate(20);//주파수를 지정 20Hz

	// try//로봇팔의 직렬 연결 시도
	// {
	// 	_serial.setPort("/dev/ttyUSB0");
	// 	_serial.setBaudrate(115200);
	// 	serial::Timeout to = serial::Timeout::simpleTimeout(1000);
	// 	_serial.setTimeout(to);
	// 	_serial.open();
	// 	_serial.write("M50\r\n");
	// 	ROS_INFO_STREAM("Port has been open successfully");
	// }
	// catch (serial::IOException& e)
	// {
	// 	ROS_ERROR_STREAM("Unable to open port");
	// 	return -1;
	// }
	
	// if (_serial.isOpen())
	// {
	// 	ros::Duration(1).sleep();				
	// 	ROS_INFO_STREAM("Attach and wait for commands");
	// }
	
	


	while(nh.ok())
	{
		tf::StampedTransform transform;
	}
	ros::spin();
	return 0;
}

