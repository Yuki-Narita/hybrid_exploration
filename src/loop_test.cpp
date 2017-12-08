#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <ros/callback_queue.h>

ros::CallbackQueue tf_queue;
ros::Subscriber tf_sub;
ros::SubscribeOptions tf_option;

float pre_loop_x = 0;
float pre_loop_y = 0;
int loop_count = 0;
const int loop_closing_max = 20;

void tf_callback(const geometry_msgs::Point::ConstPtr& tf_data){
	float trans_x = tf_data -> x;
	float trans_y = tf_data -> y;

	std::cout << "x:" << trans_x << "," << "y:" << trans_y << std::endl;
	
	if(trans_x != pre_loop_x || trans_y != pre_loop_y){
		pre_loop_x = trans_x;
		pre_loop_y = trans_y;
		loop_count++;
		std::cout << "ループクロージング" << loop_count << "回目" << std::endl;
	}
}

int main(int argc, char** argv){

	ros::init(argc, argv, "loop_test");
	ros::NodeHandle tos;

	tf_option = ros::SubscribeOptions::create<geometry_msgs::Point>("/odom2map_support",1,tf_callback,ros::VoidPtr(),&tf_queue);
	tf_sub = tos.subscribe(tf_option);

	while(ros::ok()){
		tf_queue.callOne(ros::WallDuration(1));
		std::cout << "loop_count : " << loop_count << std::endl;
		if(loop_count == loop_closing_max){
			std::cout << "ループクロージングを" << loop_closing_max << "回したので終了" << std::endl;
			break;
		}
	}

	return 0;
}
