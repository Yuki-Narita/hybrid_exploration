#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <nav_msgs/OccupancyGrid.h>

ros::CallbackQueue rmtc_queue;
ros::Subscriber rmtc_sub;
ros::SubscribeOptions rmtc_option;
nav_msgs::OccupancyGrid rmtc_map;
ros::Publisher rmtc_pub;

void rmtc_callback(const nav_msgs::OccupancyGrid::ConstPtr& rmtc_msg){
	rmtc_map.header = rmtc_msg -> header;
	rmtc_map.info = rmtc_msg -> info;
	rmtc_map.data = rmtc_msg ->data;
	
}

int main(int argc, char** argv){

	ros::init(argc, argv, "ros1robot_maptopic_converter");//ロボットによって書き換え
	ros::NodeHandle rmtc;

	rmtc_pub = rmtc.advertise<nav_msgs::OccupancyGrid>("/ros1robot/converted_map", 1);//ロボットによって書き換え

	rmtc_option = ros::SubscribeOptions::create<nav_msgs::OccupancyGrid>("/ros1robot/map",1,rmtc_callback,ros::VoidPtr(),&rmtc_queue);//ロボットによって書き換え
	rmtc_sub = rmtc.subscribe(rmtc_option);

	while(ros::ok()){
		rmtc_queue.callOne(ros::WallDuration(0.05));
		rmtc_pub.publish(rmtc_map);
	}

	return 0;
}
