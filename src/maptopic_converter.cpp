#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <nav_msgs/OccupancyGrid.h>
#include <unistd.h>

ros::CallbackQueue mtc_queue;
ros::Subscriber mtc_sub;
ros::SubscribeOptions mtc_option;
nav_msgs::OccupancyGrid mtc_map;
ros::Publisher mtc_pub;

void mtc_callback(const nav_msgs::OccupancyGrid::ConstPtr& mtc_msg){
	mtc_map.header = mtc_msg -> header;
	mtc_map.info = mtc_msg -> info;
	mtc_map.data = mtc_msg ->data;
	
}

int main(int argc, char** argv){

	ros::init(argc, argv, "ros1robot_maptopic_converter");//ロボットによって書き換え
	ros::NodeHandle mtc;

	mtc_pub = mtc.advertise<nav_msgs::OccupancyGrid>("/ros1robot/map", 1);//ロボットによって書き換え

	mtc_option = ros::SubscribeOptions::create<nav_msgs::OccupancyGrid>("/map",1,mtc_callback,ros::VoidPtr(),&mtc_queue);
	mtc_sub = mtc.subscribe(mtc_option);

	while(ros::ok()){
		mtc_queue.callOne(ros::WallDuration(0.05));
		mtc_pub.publish(mtc_map);
	}

	return 0;
}
