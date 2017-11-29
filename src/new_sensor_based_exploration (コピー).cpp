#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <ros/callback_queue.h>
#include <fstream>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf2_msgs/TFMessage.h>
#include <visualization_msgs/Marker.h>
#include <kobuki_msgs/BumperEvent.h>
#include <geometry_msgs/Point.h>


/*パブリッシュ、サブスクライブ関連*/
ros::CallbackQueue VFH_queue;
ros::CallbackQueue Branch_queue;
ros::CallbackQueue odom_queue;
ros::CallbackQueue scan_branch_queue;

ros::SubscribeOptions VFH_option;
ros::SubscribeOptions Branch_option;
ros::SubscribeOptions odom_option;
ros::SubscribeOptions scan_branch_option;

ros::Subscriber VFH_sub;
ros::Subscriber Branch_sub;
ros::Subscriber odom_sub;
ros::Subscriber scan_branch_sub;

ros::CallbackQueue scan_queue;
ros::SubscribeOptions scan_option;
ros::Subscriber scan_sub;

ros::Publisher vel_pub;
ros::Publisher minus_vel_pub;
ros::Publisher marker_pub;

geometry_msgs::Twist vel;
geometry_msgs::Twist minus_vel;

ros::CallbackQueue bumper_queue;
ros::SubscribeOptions bumper_option;
ros::Subscriber bumper_sub;

ros::CallbackQueue tf_queue;
ros::SubscribeOptions tf_option;
ros::Subscriber tf_sub;


uint32_t list = visualization_msgs::Marker::LINE_LIST;
visualization_msgs::Marker marker3;

/*グローバルで使う変数*/
ros::Time set_time;//時間制限系while文の開始時間
float goal_x;//分岐領域までの距離X(ロボットの前方向)
float goal_y;//分岐領域までの距離X(ロボットの横方向)

float goal_point_x;
float goal_point_y;

float vfh_gra_x;
float vfh_gra_y;

float gra_angle;
float gra_angle_r;

int center_count = 0;

float goal_angle;//分岐領域がある角度(表示用)
float odom_x;//オドメトリx
float odom_y;//オドメトリy
double yaw;//ヨー角
std::vector<float> odom_log_x;//オドメトリxの履歴を保存
std::vector<float> odom_log_y;//オドメトリyの履歴を保存
double pre_rotate;//前回の分岐に回転した方向

int loop_count = 0;
float pre_loop_x = 0;
float pre_loop_y = 0;
ros::Time start;

float pre_theta = 0;

const float safe_space = 0.45;//ロボットの直径(VFHでこの値以上に空間があれば安全と判断)[m]
const float side_dis = 0.375;//VFH_curveの横方向制限
const float origin_dis = 0.375;//VFH_curveの横方向制限
int which_bumper = 0;
bool bumper_hit = false;

/*処理で用いる定数パラメータ*/
//基本パラメータ///
const float PI = 3.1415926;//円周率π
const float forward_vel = 0.2;//前進速度[m/s]
const float rotate_vel = 0.5;//回転速度[rad\s]
//VFH関連のパラメータ///
const float scan_threshold = 0.5;//VFHでの前方の安全確認距離(この距離以内に障害物がなければ安全と判断)[m]
const float robot_diameter = 0.40;//ロボットの直径(VFHでこの値以上に空間があれば安全と判断)[m]
const float forward_dis = 0.5;//一回のVFHで前方向に進む距離[m]
const int div_num = 2;//VFHでカーブを行うときに目的地までの距離を分割する数(偶数)
const float back_vel = -0.2;//VFHで全部nanだったときの後退速度[m/s]
const float back_time = 0.5;//VFHで全部nanだったときに後退する時間[s]
const float not_exist_rotate_time = 0.2;//VFHで全部nanだったときに回転する時間[s]
const int go_back_odom = 6;//VFHで全部nanのときの回転方向計算で進行方向を求めるときのオドメトリを遡る個数+1
//分岐領域関連のパラメータ///
const float Branch_threshold = 1.0;//分岐領域の判断をする距離差の閾値[m]
const float Branch_range_limit = 5.0;//分岐領域の判断を行う距離の最大値(分岐がこの値以上遠くにあっても認識しない)[m]
const float branch_obst_limit = 1.0;//スキャンデータの中心がこの値以下のとき分岐領域を検出しない[m]
const float fix_sensor = 0.07;//分岐領域座標設定のときにセンサーから取れる距離の誤差を手前に補正(センサー値からマイナスする)[m]
const float rotation_error = 0.09*6;//分岐方向への回転で微妙に回転しすぎるため補正をかける(この値だけ回転角を減らす)(5[deg]*定数)[rad]
const float after_rotate_forward = 1.5;//回転直後に壁があった時に角度を戻したあとどれくらい進むか[m]
//重複探査関連のパラメータ///
const float duplication_margin = 0.3;//重複探査の判断をするときの半径[m]←正方形の辺の半分の長さでした




const float right_angle = (PI/2) - rotation_error;//分岐領域への回転などで使う直角の角度
const float defalt_rotate_yaw = 0.87;//分岐方向への回転をセンサデータから行うときの最小限回る角度
const float scan_branch_limit = 1.5;//分岐方向への回転をセンサデータから行うときにこの値以上だったら数値があっても良い
const float branch_y_threshold = 2.0;//分岐領域の2点間のｙ座標の差がこの値以下のとき分岐として検出
const float nothingness_vel = 0.01;//虚無の速度[m/s]

float scan_angle;//この角度の範囲内に空間があれば回転を終了する

const float branch_rotate_scan_angle = 0.09;//スキャンデータから分岐回転終了を決定のに必要な空間の角度、この値の±を見る[rad]
const float branch_angle = 0.04;//分岐領域を検出するのに必要な障害物がない空間の角度
const float scan_branch_end = PI/2 + 0.17;//スキャンデータからの分岐回転終了を決めるときの最大回転角
const float obst_recover_angle = 0.09;//リカバリー回転のときこの角度の±の範囲に障害物がなければ回転終了
const int loop_closing_max = 10;//プログラムを切り替えるために必要なループクロージングの回数

/*判別用フラグ*/
bool AI_wakeup = false;//AIの起動演出をするかどうか
bool branch_find_flag = false;//分岐領域があるかどうか
bool need_back = true;//全部nanだったときに最初だけバックする
bool need_rotate_calc = true;//全部nanだったときの回転方向を計算する必要があるか
bool first_move = true;//多分最初の移動だけ摩擦の関係で速度がちゃんと送れないので補正する
bool after_rotate = false;//分岐方向に回転した直後壁があったらもう少し進む
bool scan_rotation_ok = false;//スキャンデータからの分岐回転を終了していいか


/*functions*/
void VFH_scan_callback(const sensor_msgs::LaserScan::ConstPtr& VFH_msg);//VFHで移動方向を決定する
void approx(std::vector<float> &scan);//VFHのときに使うスキャンデータのnanを線形近似で補正
float VFH_move_angle(std::vector<float> &ranges, float angle_min, float angle_increment, float all_nan);//VFHの移動角を計算
void vel_after_rotate();//分岐方向に回転した直後にVFHで全部nanだったときの特殊処理
void vel_recovery();//VFHで全部nanだったときのリカバリ処理
float rotation_direction();//VFHで全部nanだったときに回転する向きを決定する
void vel_curve_VFH(float rad_min);//VFHで移動する速度を送る
void Branch_area_callback(const sensor_msgs::LaserScan::ConstPtr& Branch_msg);//分岐領域の処理を行う
void Branch_search(std::vector<float> &fixed_ranges,std::vector<float> &fixed_angle);//分岐領域の検索
void VFH4vel_publish_Branch();//分岐領域へ向かう命令を送る
void duplicated_point_detection();//重複探査の判断を行う
//void odom_callback(const nav_msgs::Odometry::ConstPtr& odom_msg);//オドメトリを取得
void AI_Wakeup_Performance();//AIの起動演出
void branch_rotate_odom(double y, float d_rot, float rotation_angle);//オドメトリから取得したヨー角の変化が一定値(設定済み)を超えるまで回転する
void branch_forward(float start_x, float start_y, float goal);//指定した距離だけVFHで進む
void branch_rotate_scan();//分岐方向への回転をスキャンデータ
void scan_branch_callback(const sensor_msgs::LaserScan::ConstPtr& scan_Branch_msg);//スキャンデータの中心の値から回転終了の判断をする
void nothingness_input();//虚無の入力

/*
void export_data2(float i, float range){
	std::ofstream ofs("scan_resize_test.csv",std::ios::app);
	ofs << i << "," << range << std::endl;
}
*/

void odom_marking(float x, float y){
	geometry_msgs::Point marking;
	marking.x = x;
	marking.y = y;
	marking.z = 0.0;
	marker3.points.push_back(marking);
	marker_pub.publish(marker3);
}

void display_goal_angle(float x, float y){
	uint32_t arrow = visualization_msgs::Marker::ARROW;
	visualization_msgs::Marker marker1;
	marker1.header.frame_id = "map";
	marker1.header.stamp = ros::Time::now();
	marker1.ns = "basic_shapes";
	marker1.id = 0;
	marker1.type = arrow;
	marker1.action = visualization_msgs::Marker::ADD;
	geometry_msgs::Point pgo0;
	geometry_msgs::Point pgo1;

	odom_queue.callOne(ros::WallDuration(1));
	
	pgo0.x = odom_x;
	pgo0.y = odom_y;
	pgo0.z = 0.0;

	pgo1.x = x;
	pgo1.y = y;
	pgo1.z = 0.0;

	marker1.points.push_back(pgo0);
	marker1.points.push_back(pgo1);

	marker1.scale.x = 0.1;
	marker1.scale.y = 0.5;
	marker1.scale.z = 0.0;

	marker1.color.r = 0.0f;
	marker1.color.g = 1.0f;
	marker1.color.b = 0.0f;
	marker1.color.a = 1.0;

	marker1.lifetime = ros::Duration(0);

	marker_pub.publish(marker1);
}

void display_gravity(float x, float y){
	uint32_t line = visualization_msgs::Marker::LINE_STRIP;
	visualization_msgs::Marker marker2;
	marker2.header.frame_id = "map";
	marker2.header.stamp = ros::Time::now();
	marker2.ns = "basic_shapes";
	marker2.id = 1;
	marker2.type = line;
	marker2.action = visualization_msgs::Marker::ADD;
	marker2.pose.orientation.w = 1.0;
	geometry_msgs::Point pgr0;
	geometry_msgs::Point pgr1;

	odom_queue.callOne(ros::WallDuration(1));

	pgr0.x = odom_x;
	pgr0.y = odom_y;
	pgr0.z = 0.0;

	pgr1.x = x;
	pgr1.y = y;
	pgr1.z = 0.0;

	marker2.points.push_back(pgr0);
	marker2.points.push_back(pgr1);

	marker2.scale.x = 0.1;

	marker2.color.r = 1.0f;
	marker2.color.g = 0.0f;
	marker2.color.b = 0.0f;
	marker2.color.a = 1.0;

	marker2.lifetime = ros::Duration(0);

	marker_pub.publish(marker2);
}



void export_data(float i, float range){
	std::ofstream ofs("odom.csv",std::ios::app);
	ofs << i << "," << range << std::endl;
}


void tf_callback(const tf2_msgs::TFMessage::ConstPtr& tf_data){
	float trans_x = tf_data -> transforms[0].transform.translation.x;
	float trans_y = tf_data -> transforms[0].transform.translation.y;
	std::basic_string<char> child_frame_id = tf_data -> transforms[0].child_frame_id;
	std::basic_string<char> frame_id = tf_data -> transforms[0].header.frame_id;


	if(frame_id == "map"){
		std::cout << "x:" << trans_x << "," << "y:" << trans_y << std::endl;
		//std::cout << "child_frame_id : " << child_frame_id << std::endl;
		//std::cout << "frame_id : " << frame_id  << "\n" << std::endl;
		if(trans_x != pre_loop_x || trans_y != pre_loop_y){
			pre_loop_x = trans_x;
			pre_loop_y = trans_y;
			loop_count++;
			std::cout << "ループクロージング" << loop_count << "回目" << std::endl;
		}
	}
}




void nothingness_input(){
	vel.angular.z = 0;
	vel.linear.x = 0;
	vel_pub.publish(vel);
	vel.angular.z = 0;
	vel.linear.x = nothingness_vel;
	vel_pub.publish(vel);
	vel.angular.z = 0;
	vel.linear.x = 0;
	vel_pub.publish(vel);

	//sleep(1.0);
}


void scan_branch_callback(const sensor_msgs::LaserScan::ConstPtr& scan_Branch_msg){
	std::vector<float> ranges = scan_Branch_msg->ranges;
	float angle_increment = scan_Branch_msg->angle_increment;

	int scan_width = scan_angle/angle_increment;//99

	int scan_i_min = (ranges.size()/2)-1 - scan_width;
	int scan_i_max = (ranges.size()/2) + scan_width + 1;

	for(int i = scan_i_min;i<scan_i_max;i++){
		//if(isnan(ranges[i]) || ranges[i] > scan_branch_limit){//
		if(!isnan(ranges[i]) && ranges[i] < scan_branch_limit){
			scan_rotation_ok = false;
			std::cout << "rotation_return(debag)" << std::endl;
			return;
		}
	}

	scan_rotation_ok = true;
	std::cout << "rotation_true(debag)" << std::endl;

	//scan_rotation_ok = true;

	/*if(isnan(ranges[ranges.size()/2]) || ranges[ranges.size()/2] > scan_branch_limit){
		scan_rotation_ok = true;
	}*/

}




//分岐方向への回転をスキャンで決める
void branch_rotate_scan(){

	double y;	
	double y2;
	double old_y2;
	bool change_sign = false;
	bool if_finish = true;

	odom_queue.callOne(ros::WallDuration(1));

	y = yaw;

	std::cout << "スキャンデータから回転の終了を決定" << std::endl;

	//スキャンデータの中心に値が出ないもしくは閾値以上に大きい値がくるまで回る
	//分岐を見つけた時点では中心が条件を満たしているので最初に少しは回転させないとだめかも
	branch_rotate_odom(y,goal_y,defalt_rotate_yaw);//最小限の回転

	odom_queue.callOne(ros::WallDuration(1));
	y2 = yaw;

	scan_angle = branch_rotate_scan_angle;

	while(!scan_rotation_ok && ros::ok()){
		scan_branch_queue.callOne(ros::WallDuration(1));
		if(!scan_rotation_ok){
			vel_pub.publish(vel);
		}
		//scan_rotation_ok = true;
		if(!change_sign){
			old_y2 = y2;
		}
		odom_queue.callOne(ros::WallDuration(1));
		y2 = yaw;
		std::cout << y << "," << y2 << "," << old_y2  << "," << change_sign << "(debag)" << std::endl;
		if(vel.angular.z < 0){
			if(y2 - old_y2 > PI){
				change_sign = true;
				y2 = -2*PI + y2; 
			}
		}
		else{
			if(y2 -old_y2 < -PI){
				change_sign = true;
				y2 = 2*PI + y2; 
			}
		}
		std::cout << y << "," << y2 << "," << old_y2 << "," << change_sign << "(debag)" << std::endl;
		std::cout << "*****分岐領域発見時からの回転角度 " << y2-y << " [rad] *****" << std::endl;
		if(if_finish && std::abs(y2-y) > scan_branch_end){
			std::cout << "逆回転" << std::endl;
			vel.angular.z *= -1;
			if_finish = false;
		}
	}
	scan_rotation_ok = false;
}

//分岐方向への回転をオドメトリで決める
void branch_rotate_odom(double y, float d_rot, float rotation_angle){
	//const float rotation_angle = (PI/2) - rotation_error;
	double y2;
	double old_y2;
	bool change_sign = false;

	odom_queue.callOne(ros::WallDuration(1));
	
	y2 = yaw;

	vel.linear.x = 0;

	if(d_rot < 0){
		std::cout << "-(debag)" << std::endl;
		vel.angular.z = -rotate_vel;
		std::cout << goal_y << "," << y << "," << y2 << "(debag)" << std::endl;
		while((-rotation_angle - (y2-y)) < 0 && ros::ok()){
			vel_pub.publish(vel);

			if(!change_sign){
				old_y2 = y2;
			}	
			odom_queue.callOne(ros::WallDuration(1));
			y2 = yaw;
			if(y2 - old_y2 > PI){
				change_sign = true;
				y2 = -2*PI + y2; 
			}
			std::cout << "*****分岐領域発見時からの回転角度 " << y2-y << " [rad] *****" << std::endl;
		}
	}
	else{
		std::cout << "+(debag)" << std::endl;
		vel.angular.z = rotate_vel;
		std::cout << goal_y << "," << y << "," << y2 << "(debag)" << std::endl;
		while((rotation_angle - (y2-y)) > 0 && ros::ok()){
			vel_pub.publish(vel);

			if(!change_sign){
				old_y2 = y2;
			}
			odom_queue.callOne(ros::WallDuration(1));
			y2 = yaw;

			if(y2 -old_y2 < -PI){
				change_sign = true;
				y2 = 2*PI + y2; 
			}
			std::cout << "*****分岐領域発見時からの回転角度 " << y2-y << " [rad] *****" << std::endl;
		}
	}
}

//分岐まで前進(goalで指定した距離だけ前進)
void branch_forward(float start_x, float start_y, float goal){

	float odom_abs = 0;


	std::cout << "(" << start_x << "," << start_y << ")" << std::endl;	

	std::cout << "***** スタート (" << odom_abs << ") , ゴール  ("<< goal_x << ") *****" << std::endl;
	
	while ((goal - odom_abs) > 0 && ros::ok()){

		VFH_queue.callOne(ros::WallDuration(1));

		std::cout << "(" << odom_x << "," << odom_y << ")" << std::endl;	

		odom_abs = std::max(std::abs(odom_x - start_x),std::abs(odom_y - start_y));

		std::cout << "***** 現在地  (" << odom_abs << ") , ゴール  ("<< goal << ") *****" << std::endl;
   	 }
}

void vel_after_rotate(){
	double y;

	odom_queue.callOne(ros::WallDuration(1));

	y = yaw;

	std::cout << "分岐方向に障害物があったのでもう少し回ります" << std::endl;
	
	scan_angle = obst_recover_angle;
	//vel.angular.z *= -1;

	while(!scan_rotation_ok && ros::ok()){
		scan_branch_queue.callOne(ros::WallDuration(1));//scan_angleに設定した角度の範に空間ができるまで回転する
		if(!scan_rotation_ok){
			vel_pub.publish(vel);
		}
	}

	scan_rotation_ok = false;

	//branch_rotate_odom(y,pre_rotate,right_angle);
	//回転終了の判断に使う角度を渡してその間回ってくれる関数

	std::cout << "はい!OK" << std::endl;
}

void Branch_search(std::vector<float> &fixed_ranges,std::vector<float> &fixed_angle){
	int i;
	float robot_x;
	float robot_y;
	float next_robot_x;
	float next_robot_y;
	float Branch_dist;
	float Branch_area;
	float temp_Branch_center_dist;
	float Branch_center_dist = 10000000000.0;
	float tmp_goal_x;
	float tmp_goal_y;
	float tmp_goal_angle;
	
	float debag_robot_x = 0;
	float debag_robot_y = 0;
	float debag_next_robot_x = 0;
	float debag_next_robot_y = 0;

	goal_x = 0;
	goal_y = 0;
	goal_angle = 0;

	for(i=0;i<fixed_ranges.size()-1;i++){
		robot_x = fixed_ranges[i]*cos(fixed_angle[i]);
		next_robot_x = fixed_ranges[i+1]*cos(fixed_angle[i+1]);
		Branch_dist = std::abs(next_robot_x - robot_x);
		if(Branch_dist >= Branch_threshold){
			robot_y = fixed_ranges[i]*sin(fixed_angle[i]);
			next_robot_y = fixed_ranges[i+1]*sin(fixed_angle[i+1]);
			temp_Branch_center_dist = std::abs(next_robot_x - robot_x) + std::abs(next_robot_y - robot_y);
			if(temp_Branch_center_dist < Branch_center_dist){
				Branch_center_dist = temp_Branch_center_dist;
				tmp_goal_x = (next_robot_x + robot_x)/2;
				tmp_goal_y = (next_robot_y + robot_y)/2;
				tmp_goal_angle = (fixed_angle[i] + fixed_angle[i+1])/2;
				//if(tmp_goal_angle = 
				if(robot_x < Branch_range_limit && next_robot_x < Branch_range_limit){
				//if(tmp_goal_x < Branch_range_limit){
					if(std::abs(tmp_goal_y) < branch_y_threshold){
						//branch_y_threshold
						debag_robot_x = robot_x;
						debag_robot_y = robot_y;
						debag_next_robot_x = next_robot_x;
						debag_next_robot_y = next_robot_y;
						//
						goal_x = tmp_goal_x;
						goal_y = tmp_goal_y;
						goal_angle = tmp_goal_angle;
						branch_find_flag = true;
					}
				}
			}
		}		
	}
	
	goal_x = goal_x - fix_sensor;

	std::cout << "robot_x: " << debag_robot_x << std::endl;
	std::cout << "robot_y: " << debag_robot_y << std::endl;
	std::cout << "next_robot_x: " << debag_next_robot_x << std::endl;
	std::cout << "next_robot_y: " << debag_next_robot_y << std::endl;
}

float VFH_move_angle(std::vector<float> &ranges, float angle_min, float angle_increment, float all_nan, std::vector<float> &angles){
	float rad_start;
	float rad_end;
	float rad_diff;
	float rad_chord;
	float rad_center;
	float rad_center_abs;
	float rad_min_abs = all_nan;
	float rad_min = all_nan;
	int i;
	float rad_gra_diff_abs;
	float rad_gra_diff;
	float rad_c_g;
	float rad_c_g_deb;
	float over_rad;
	//float gra_angle_r;
	int near_i;
	float gra_angle_abs = 3.14;
	float gra_angle_diff;
	float tmp_safe_num;
	int safe_num;
	int rad_counter = 0;
	int start_i;
	int j;
	int plus_rad_i = angles.size();
	int minus_rad_i = angles.size();
	float pd;
	float md;
	float x_g;
	float y_g;
	float ang_g;
	

	for(i=0;i<ranges.size();i++){
		//export_data(angle_min+(angle_increment*i),ranges[i]);
		if(isnan(ranges[i])){
			if(isnan(ranges[i+1])){
				ranges[i] = 0;
			}
			else{
				ranges[i] = ranges[i+1];
			}
		}
		if(ranges[i] >= scan_threshold){
			ranges[i] = scan_threshold;		
		}
	}

//gra_angleを中心にして±に角度を見ていって先にロボットの直径分の空間が見つかったらその時点でその中心を目標にする
//gra_angleをロボットの座標軸で表すgra_angle_r
	/*
	gra_angle_r = gra_angle - yaw;
	
	if(gra_angle_r < -PI){
		gra_angle_r = 2*PI + gra_angle_r;
	}

	if(gra_angle_r > PI){
		gra_angle_r = -2*PI + gra_angle_r;
	}
	
//gra_angle_rと角度が最も近くなるiの番号を調べる
	for(i=0;i<angles.size();i++){
		gra_angle_diff = std::abs(gra_angle_r - angles[i]);
		if(gra_angle_diff < gra_angle_abs){
			gra_angle_abs = gra_angle_diff;
			near_i = i;
		}
	}
	*/

	near_i = angles.size()/2 - center_count;

	if(center_count == 0){
		center_count = 1;
	}
	else{
		center_count = 0;
	}

//安全な角度を作るために必要な個数
	
	//tmp_safe_num = (asin((safe_space)/(2*scan_threshold))) / angle_increment ;
	tmp_safe_num = (asin((safe_space)/(2*forward_dis))) / angle_increment ;
	safe_num = tmp_safe_num;
	std::cout << "angle_increment:" << angle_increment << ",scan_threshold:" << scan_threshold << std::endl;
//gra_angle_rに一番近いiから±に安全角度を見つける,プラス側に行って半径分領域があったらそのマイナス側に半径分の領域があるかを見る
	//先にプラス側を見る
	for(i=near_i;i<angles.size();i++){
		//std::cout << "plusのfor" << std::endl;
		if(ranges[i] == scan_threshold){
			start_i = i;
			rad_counter = 0;
			while(ranges[i] == scan_threshold && rad_counter < safe_num && i < angles.size()-1){//iの限界
				rad_counter++;
				i++;
			}
			if(rad_counter == safe_num && start_i >= safe_num){//プラス側はOKなのでマイナス側を見に行く
				rad_counter = 0;
				for(j=start_i;j>start_i-safe_num;j--){
					if(ranges[i] == scan_threshold && rad_counter < safe_num){
						rad_counter++;
					}
					//std::cout << "j:" << j << std::endl;
				}
				if(rad_counter == safe_num){
					plus_rad_i = start_i;
					//std::cout << "breakした" << std::endl;
					break;
				}	
			}
		}
		else{
			i++;
		}
	}
	//std::cout << "plusのfor抜けた" << std::endl;




	for(i=near_i;i>0;i--){
		//std::cout << "minusのfor" << std::endl;
		if(ranges[i] == scan_threshold){
			start_i = i;
			rad_counter = 0;
			while(ranges[i] == scan_threshold && rad_counter < safe_num && i > 1){
				rad_counter++;
				i--;
			}
			if(rad_counter == safe_num && start_i <= angles.size()-safe_num){//マイナス側はOKなのでプラス側を見に行く//ここやってない//やった
				rad_counter = 0;
				for(j=start_i;j<start_i+safe_num;j++){
					if(ranges[i] == scan_threshold && rad_counter < safe_num){
						rad_counter++;
					}
				}
				if(rad_counter == safe_num){
					minus_rad_i = start_i;
					//std::cout << "breakした" << std::endl;
					break;
				}	
			}
		}
		else{
			i--;
		}
	}
	//std::cout << "minusのfor抜けた" << std::endl;

	
	
	//plusとminusでnear_iから近い方を選択してrad_minに入れる

	if(plus_rad_i != angles.size() || minus_rad_i != angles.size()){
		pd = std::abs(angles[near_i] - angles[plus_rad_i]);
		md = std::abs(angles[near_i] - angles[minus_rad_i]);

		if(plus_rad_i == angles.size()){
			pd = 100;
		}

		if(minus_rad_i == angles.size()){
			md = 100;
		}

		if(pd<=md){
			rad_min = angles[plus_rad_i];
		}
		else{
			rad_min = angles[minus_rad_i];
		}


		//目標角度をグローバルで
		ang_g = yaw + rad_min;
		if(ang_g > PI){
			over_rad = ang_g - PI;
			ang_g = -PI + over_rad;
		}
		if(ang_g < -PI){
			over_rad = ang_g + PI;
			ang_g = PI - over_rad;
		}

		odom_queue.callOne(ros::WallDuration(1));

		x_g = scan_threshold * cos(ang_g) + odom_x;
		y_g = scan_threshold * sin(ang_g) + odom_y;

		display_goal_angle(x_g, y_g);	

	}

	std::cout << "gra_angle:" << gra_angle << std::endl;
	std::cout << "yaw:" << yaw << std::endl;
	std::cout << "gra_angle_r:" << gra_angle_r << std::endl;
	std::cout << "near_i:" << near_i  << ", near_i_angle:" << angles[near_i] << std::endl;
	std::cout << "safe_num:" << safe_num << std::endl;
	std::cout << "plus_i: " << plus_rad_i << ", plus_i_rad:" << angles[plus_rad_i] << std::endl;
	std::cout << "minus_i: " << minus_rad_i << ", minus_i_rad:" << angles[minus_rad_i] << std::endl;
	std::cout << "rad_min:" << rad_min << std::endl;


	return rad_min;
}

/*
float VFH_move_angle(std::vector<float> &ranges, float angle_min, float angle_increment, float all_nan){
	float rad_start;
	float rad_end;
	float rad_diff;
	float rad_chord;
	float rad_center;
	float rad_center_abs;
	float rad_min_abs = all_nan;
	float rad_min = all_nan;
	int i;
	

	for(i=0;i<ranges.size();i++){
		//export_data(angle_min+(angle_increment*i),ranges[i]);
		if(isnan(ranges[i])){
			if(isnan(ranges[i+1])){
				ranges[i] = 0;
			}
			else{
				ranges[i] = ranges[i+1];
			}
		}
		if(ranges[i] >= scan_threshold){
			ranges[i] = scan_threshold;		
		}
	}

	i = 0;

	while(i<ranges.size() && ros::ok()){
		if(ranges[i] == scan_threshold){
			rad_start = angle_min+(angle_increment*i);
			while(ranges[i] == scan_threshold && ros::ok()){
				i++;		
			}
			rad_end = angle_min+(angle_increment*(i-1));
			
			rad_diff = rad_end - rad_start;
			
			rad_chord = 2 * scan_threshold * sin(rad_diff/2);

			if(rad_chord >= robot_diameter){
				rad_center = (rad_start+rad_end)/2;
				rad_center_abs = std::abs(rad_center);
				if(rad_center_abs<rad_min_abs){
					rad_min_abs = rad_center_abs;
					rad_min = rad_center;
				}
			}			
		}
		else{
			i++;
		}
	}


	
	return rad_min;
}
*/
void AI_Wakeup_Performance(){
	std::cout << "「Sensor-Based Exploration Proglam Start」" << std::endl;
	sleep(0.25);
	std::cout << "・・・・・・" << std::endl;
	sleep(0.25);
	std::cout << "・・・・・・" << std::endl;
	sleep(0.5);
	std::cout << "こんちにわ!!地図作成AIちゃんだよ〜" << std::endl;
	sleep(0.5);
	std::cout << "これから障害物回避をしながら地図を作っていくね" << std::endl;
	sleep(0.5);
	std::cout << "それじゃあさっそく地図作成・・・スタート!!" << std::endl;
	sleep(0.5);
}

float rotation_direction(){
	int odom_num = odom_log_x.size();
	float oldest_x = odom_log_x[odom_num-go_back_odom];
	float oldest_y = odom_log_y[odom_num-go_back_odom];
	float var_x;
	float var_y;
	float abs_var_x;
	float abs_var_y;


	//xとｙのどちらが大きく変動したかを確認
	var_x = odom_x - oldest_x;
	var_y = odom_y - oldest_y;

	abs_var_x = std::abs(var_x);
	abs_var_y = std::abs(var_y);
/*
	std::cout << "回転セレクトタイム" << std::endl;
	std::cout << odom_num << std::endl;
	std::cout << odom_x << "," << odom_y << "," << oldest_x << "," << oldest_y  << std::endl;
	std::cout << var_x << "," << var_y << "," << abs_var_x << "," << abs_var_y << std::endl;
*/
	if(abs_var_x >= abs_var_y){
		if(var_x * var_y >= 0){
			//std::cout << "x-" << std::endl;
			return -rotate_vel;
		}
		else{
			//std::cout << "x+" << std::endl;
			return rotate_vel;
		}
	}
	else{
		if(var_x * var_y >= 0){
			//std::cout << "y+" << std::endl;
			return rotate_vel;
		}
		else{
			//std::cout << "y-" << std::endl;
			return -rotate_vel;
		}
	}
}

void odom_callback(const geometry_msgs::Point::ConstPtr& odom_msg){
	odom_x = odom_msg -> x;
	odom_y = odom_msg -> y;
	yaw = odom_msg -> z;
}

void vel_recovery(){
	if(need_back){
		std::cout << "前に進めないよ;;" << std::endl;

		vel.angular.z = 0;
		vel.linear.x = back_vel;
		ros::Duration duration(back_time);
	
		std::cout << "バック♪バック♪" << std::endl;

		set_time = ros::Time::now();	
	
		while(ros::Time::now()-set_time < duration){
			vel_pub.publish(vel);
		}
		need_back = false;
	}
		
	std::cout << "かいて〜ん!!" << std::endl;

	if(need_rotate_calc){
		if(pre_theta >=0){
			vel.angular.z = -rotate_vel;
		}
		else{
			vel.angular.z = rotate_vel;
		}
		//vel.angular.z = rotation_direction();
		vel.linear.x = 0;
		need_rotate_calc = false;
	}

	scan_angle = obst_recover_angle;

	if(bumper_hit){
		if(which_bumper == 0){
			vel.angular.z = -rotate_vel;
		}

		else if(which_bumper == 1){

			if(pre_theta > 0 ){
				vel.angular.z = -rotate_vel;
			}
			else{
				vel.angular.z = rotate_vel;
			}
		}
		
		else{
			vel.angular.z = rotate_vel;
		}	
	
		ros::Duration duration2(back_time+3.0);
		set_time = ros::Time::now();
		while(ros::Time::now()-set_time < duration2){
			vel_pub.publish(vel);
		}		

	}
	else{
		while(!scan_rotation_ok && ros::ok()){
			if(!scan_rotation_ok){
				vel_pub.publish(vel);
			}
			scan_branch_queue.callOne(ros::WallDuration(1));//scan_angleに設定した角度の範に空間ができるまで回転する
		}

		scan_rotation_ok = false;
	}
	//ros::Duration duration(not_exist_rotate_time);

	/*set_time = ros::Time::now();	
	while(ros::Time::now()-set_time < duration){
		vel_pub.publish(vel);
	}*/
}


void vel_curve_VFH(float rad_min ,float angle_max){
	const float theta = rad_min;
	const float v = forward_vel;
	
	float y = origin_dis*pow(cos(rad_min),2);
	bool d = true;	
	
	if(std::abs(rad_min) > (angle_max/2)){
		y = side_dis * cos(rad_min) / std::abs(sin(rad_min));
		std::cout << "y_side: " << y << std::endl;
		d = false;
	}
	float y_div = y/div_num;
	float rho;
	float theta_rho;
	float omega;
	float t;

	pre_theta = theta;

	theta_rho = 2*theta;
	rho = y_div/sin(theta_rho);
	omega = v/rho;
	t = theta_rho/omega;

	vel.linear.x = v;
	vel.angular.z = omega;

	std::cout << theta << "(theta_debag)" << std::endl;
	std::cout << t << "(t_debag)" << std::endl;
	std::cout << vel.linear.x << "(v_debag)" << std::endl;
	std::cout << vel.angular.z << "(o_debag)" << std::endl;

	for(int i=0;i<(div_num/2);i++){
		for(int k=0;k<1;k++){
			vel_pub.publish(vel);
		}
		std::cout << "障害物を回避しながら移動中♪" << std::endl;
	}
}

/*
void vel_curve_VFH(float rad_min){
	const float theta = rad_min;
	const float y = forward_dis;
	const float v = forward_vel;
	
	float y_div = y/div_num;
	float rho;
	float theta_rho;
	float omega;
	float t;
	float t_plus;

	pre_theta = theta;

	theta_rho = 2*theta;
	rho = y_div/sin(theta_rho);
	omega = v/rho;
	t = theta_rho/omega;

	//if(first_move){
	if(true){
		t_plus = t + 0.6;//最初に送る速度命令はdelayがあるので時間を足す(いらない？
		first_move = false;
	}
	else{
		t_plus = t;
	}	


	//虚無のパブリッシュ///
	nothingness_input();

	vel.linear.x = v;
	minus_vel.linear.x = v;
	vel.angular.z = omega;
	minus_vel.angular.z = -omega;

	std::cout << theta << "(theta_debag)" << std::endl;
	std::cout << t << "(t_debag)" << std::endl;
	
	ros::Duration duration_curve(t_plus);//時間増やしたバージョン
	ros::Duration duration_curve_minus(t);



	
	for(int i=0;i<div_num/2;i++){
		set_time = ros::Time::now();
		while(ros::Time::now()-set_time < duration_curve){
			vel_pub.publish(vel);
		}
		std::cout << "障害物を回避しながら移動中♪" << std::endl;

		//虚無のパブリッシュ///
		nothingness_input();

		set_time = ros::Time::now();
		while(ros::Time::now()-set_time < duration_curve_minus){
			vel_pub.publish(minus_vel);
		}
	}

	//オドメトリを保存//

	odom_queue.callOne(ros::WallDuration(1));

	//std::cout << "(" << odom_x << "," << odom_y << ")" << std::endl;	

	odom_log_x.push_back(odom_x);
	odom_log_y.push_back(odom_y);

	export_data(odom_x,odom_y);
	////////////////
	
}
*/

//重複探査検出用
void duplicated_point_detection(){
	float global_x;//分岐領域の世界座標
	float global_y;//分岐領域の世界座標
	float x_margin_plus;
	float x_margin_minus;
	float y_margin_plus;
	float y_margin_minus;
	bool duplication_flag = false;

	odom_queue.callOne(ros::WallDuration(1));

	global_x = odom_x+(cos(yaw)*goal_x) - (sin(yaw)*goal_y);
	global_y = odom_y+(cos(yaw)*goal_y) + (sin(yaw)*goal_x);

	x_margin_plus = global_x + duplication_margin;
	x_margin_minus = global_x - duplication_margin;
	y_margin_plus = global_y + duplication_margin;
	y_margin_minus = global_y - duplication_margin;


	std::cout << "odom_x,odom_y (" << odom_x << "," << odom_y << ")" << std::endl;
	std::cout << "goal_x,goal_y (" << goal_x << "," << goal_y << ")" << std::endl;
	std::cout << "yaw (" << yaw << ")" << std::endl;
	std::cout << "global_x,global_y (" << global_x << "," << global_y << ")" << std::endl;
	
	

	for(int i=0;i<odom_log_x.size();i++){
		//過去のオドメトリが許容範囲の中に入っているか//
		if((x_margin_plus > odom_log_x[i]) && (x_margin_minus < odom_log_x[i])){
			if((y_margin_plus > odom_log_y[i]) && (y_margin_minus < odom_log_y[i])){
				duplication_flag = true;
			}
		}

		if(duplication_flag){
			branch_find_flag = false;
			std::cout << "すでに探査した領域でした・・・ぐすん;;" << std::endl;
			return;
		}
	}
	
}

void approx(std::vector<float> &scan){
	float depth,depth1,depth2;
	depth=0;
	depth1=0;
	depth2=0;

	for(int j=0,count=0;j<scan.size()-1;j++){
		depth=scan[j];
		//|val|nan|のとき
		//ROS_INFO("(%d,%d):",i,j);

		if(!isnan(depth) && isnan(scan[j+1])){
			depth1=depth;
			count++;
			//ROS_INFO("!nan&nan(%d,%d)",j,j+1);
		}

		if(isnan(depth)){
	//|nan|nan|の区間
			if(isnan(scan[j+1])){
				count++;
				//ROS_INFO("nan&nan(%d,%d)",j,j+1);
			}
	//|nan|val|のとき
			else{
				//ROS_INFO("nan&!nan(%d,%d)",j,j+1);
				depth2=scan[j+1];
	//左端がnanのとき
				if(isnan(depth1)){
					for(int k=0;k<count+1;k++)
						scan[j-k]=depth2;//0.01;//depth2;
				}
				else{
					for(int k=0;k<count;k++)
						scan[j-k]=depth2-(depth2-depth1)/(count+1)*(k+1);
				}
				//ROS_INFO("nan|val|:nancount=%d",count);
				count=0;
			}
		}
	//右端がnanのとき
		if(j==(scan.size()-1)-1 &&isnan(scan[j+1])){
			for(int k=0;k<count;k++)
				scan[j+1-k]=depth1;//0.01;//depth1;
			//ROS_INFO("val|nan|nan|:nancount=%d",count);
			count=0;
		}
	}		
	if(isnan(scan[0])){
		scan[0] = scan[1] - (scan[2] - scan[1]);
		if(scan[0] < 0){
			scan[0] = 0;
		}
	}

}

void bumper(const kobuki_msgs::BumperEvent::ConstPtr& hit_msg){
	if(hit_msg -> state == 1){
		bumper_hit = true;
		which_bumper = hit_msg -> bumper;
		std::cout << "bumper_hit" << std::endl;
	}
	else{
		bumper_hit = false;
		std::cout << "no_bumper_hit" << std::endl;
	}
}

float VFH_move_angle_g(std::vector<float> &ranges, float angle_min, float angle_increment, float all_nan, float gra_angle, std::vector<float> &angles){
	float rad_start;
	float rad_end;
	float rad_diff;
	float rad_chord;
	float rad_center;
	float rad_center_abs;
	float rad_min_abs = all_nan;
	float rad_min = all_nan;
	int i;
	float rad_gra_diff_abs;
	float rad_gra_diff;
	float rad_c_g;
	float rad_c_g_deb;
	float over_rad;
	//float gra_angle_r;
	int near_i;
	float gra_angle_abs = 3.14;
	float gra_angle_diff;
	float tmp_safe_num;
	int safe_num;
	int rad_counter = 0;
	int start_i;
	int j;
	int plus_rad_i = angles.size();
	int minus_rad_i = angles.size();
	float pd;
	float md;
	float x_g;
	float y_g;
	float ang_g;
	

	for(i=0;i<ranges.size();i++){
		//export_data(angle_min+(angle_increment*i),ranges[i]);
		if(isnan(ranges[i])){
			if(isnan(ranges[i+1])){
				ranges[i] = 0;
			}
			else{
				ranges[i] = ranges[i+1];
			}
		}
		if(ranges[i] >= scan_threshold){
			ranges[i] = scan_threshold;		
		}
	}

//gra_angleを中心にして±に角度を見ていって先にロボットの直径分の空間が見つかったらその時点でその中心を目標にする
//gra_angleをロボットの座標軸で表すgra_angle_r
	
	gra_angle_r = gra_angle - yaw;
	
	if(gra_angle_r < -PI){
		gra_angle_r = 2*PI + gra_angle_r;
	}

	if(gra_angle_r > PI){
		gra_angle_r = -2*PI + gra_angle_r;
	}

//gra_angle_rと角度が最も近くなるiの番号を調べる
	for(i=0;i<angles.size();i++){
		gra_angle_diff = std::abs(gra_angle_r - angles[i]);
		if(gra_angle_diff < gra_angle_abs){
			gra_angle_abs = gra_angle_diff;
			near_i = i;
		}
	}
//安全な角度を作るために必要な個数
	//std::cout << "ranges[i]:" << ranges[i] << std::endl;
	//tmp_safe_num = (asin((safe_space)/(2*scan_threshold))) / angle_increment ;
	tmp_safe_num = (asin((safe_space)/(2*forward_dis))) / angle_increment ;
	safe_num = tmp_safe_num;
	std::cout << "angle_increment:" << angle_increment << ",scan_threshold:" << scan_threshold << std::endl;
//gra_angle_rに一番近いiから±に安全角度を見つける,プラス側に行って半径分領域があったらそのマイナス側に半径分の領域があるかを見る
	//先にプラス側を見る
	for(i=near_i;i<angles.size();i++){
		//std::cout << "plusのfor" << std::endl;
		if(ranges[i] == scan_threshold){
			//std::cout << "fp_ranges[i]:" << ranges[i] << std::endl;
			start_i = i;
			rad_counter = 0;
			while(ranges[i] == scan_threshold && rad_counter < safe_num && i < angles.size()-1){//iの限界
					//std::cout << "sp_ranges[i]:" << ranges[i] << std::endl;
				rad_counter++;
				i++;
			}
			if(rad_counter == safe_num && start_i >= safe_num){//プラス側はOKなのでマイナス側を見に行く
				rad_counter = 0;
				for(j=start_i;j>start_i-safe_num;j--){
					if(ranges[i] == scan_threshold && rad_counter < safe_num){
						//std::cout << "tp_ranges[i]:" << ranges[i] << std::endl;
						rad_counter++;
					}
					//std::cout << "j:" << j << std::endl;
				}
				if(rad_counter == safe_num){
					plus_rad_i = start_i;
					//std::cout << "breakした" << std::endl;
					break;
				}	
			}
		}
		else{
			i++;
		}
	}
	//std::cout << "plusのfor抜けた" << std::endl;




	for(i=near_i;i>0;i--){
		//std::cout << "minusのfor" << std::endl;
		if(ranges[i] == scan_threshold){
			//std::cout << "fm_ranges[i]:" << ranges[i] << std::endl;
			start_i = i;
			rad_counter = 0;
			while(ranges[i] == scan_threshold && rad_counter < safe_num && i > 1){
				//std::cout << "sm_ranges[i]:" << ranges[i] << std::endl;
				rad_counter++;
				i--;
			}
			if(rad_counter == safe_num && start_i <= angles.size()-safe_num){//マイナス側はOKなのでプラス側を見に行く//ここやってない//やった
				rad_counter = 0;
				for(j=start_i;j<start_i+safe_num;j++){
					if(ranges[i] == scan_threshold && rad_counter < safe_num){
						//std::cout << "tm_ranges[i]:" << ranges[i] << std::endl;
						rad_counter++;
					}
				}
				if(rad_counter == safe_num){
					minus_rad_i = start_i;
					//std::cout << "breakした" << std::endl;
					break;
				}	
			}
		}
		else{
			i--;
		}
	}
	//std::cout << "minusのfor抜けた" << std::endl;

	
	
	//plusとminusでnear_iから近い方を選択してrad_minに入れる

	if(plus_rad_i != angles.size() || minus_rad_i != angles.size()){
		pd = std::abs(angles[near_i] - angles[plus_rad_i]);
		md = std::abs(angles[near_i] - angles[minus_rad_i]);

		if(plus_rad_i == angles.size()){
			pd = 100;
		}

		if(minus_rad_i == angles.size()){
			md = 100;
		}

		if(pd<=md){
			rad_min = angles[plus_rad_i];
		}
		else{
			rad_min = angles[minus_rad_i];
		}


		//目標角度をグローバルで
		ang_g = yaw + rad_min;
		if(ang_g > PI){
			over_rad = ang_g - PI;
			ang_g = -PI + over_rad;
		}
		if(ang_g < -PI){
			over_rad = ang_g + PI;
			ang_g = PI - over_rad;
		}

		odom_queue.callOne(ros::WallDuration(1));

		x_g = scan_threshold * cos(ang_g) + odom_x;
		y_g = scan_threshold * sin(ang_g) + odom_y;

		display_goal_angle(x_g, y_g);	

	}

	std::cout << "gra_angle:" << gra_angle << std::endl;
	std::cout << "yaw:" << yaw << std::endl;
	std::cout << "gra_angle_r:" << gra_angle_r << std::endl;
	std::cout << "near_i:" << near_i  << ", near_i_angle:" << angles[near_i] << std::endl;
	std::cout << "safe_num:" << safe_num << std::endl;
	std::cout << "plus_i: " << plus_rad_i << ", plus_i_rad:" << angles[plus_rad_i] << std::endl;
	std::cout << "minus_i: " << minus_rad_i << ", minus_i_rad:" << angles[minus_rad_i] << std::endl;
	std::cout << "rad_min:" << rad_min << std::endl;


	return rad_min;
}


void vel_recovery_g(){

	float gra_recov;
	float over;

	
	if(need_back){
		std::cout << "前に進めないよ;;" << std::endl;

		vel.angular.z = 0;
		vel.linear.x = back_vel;

		ros::Duration duration(back_time);
		
	
		std::cout << "バック♪バック♪" << std::endl;

		set_time = ros::Time::now();	
	
		while(ros::Time::now()-set_time < duration){
			vel_pub.publish(vel);
		}
		need_back = false;
	}
		
	std::cout << "かいて〜ん!!" << std::endl;

	odom_queue.callOne(ros::WallDuration(1));

	gra_recov = gra_angle - yaw;

	if(gra_recov > PI){
		over = gra_recov - PI;
		gra_recov = -PI + over;
	}

	if(gra_recov < -PI){
		over = gra_recov + PI;
		gra_recov = PI + over;
	}


	std::cout << "gra_angle_r: " << gra_angle_r << std::endl;
	std::cout << "yaw: " << yaw << std::endl;
	std::cout << "gra_recov: " << gra_recov << std::endl;

	if(need_rotate_calc){
		if(gra_recov >=0){
			vel.angular.z = rotate_vel;
		}
		else{
			vel.angular.z = -rotate_vel;
		}


		need_rotate_calc = false;
	}

	vel.linear.x = 0;

	scan_angle = obst_recover_angle;

	if(bumper_hit){
		if(which_bumper == 0){
			vel.angular.z = -rotate_vel;
		}

		else if(which_bumper == 1){

			if(pre_theta > 0 ){
				vel.angular.z = -rotate_vel;
			}
			else{
				vel.angular.z = rotate_vel;
			}
		}
		
		else{
			vel.angular.z = rotate_vel;
		}	
	
		ros::Duration duration2(back_time+3.0);
		set_time = ros::Time::now();
		while(ros::Time::now()-set_time < duration2){
			vel_pub.publish(vel);
		}		

	}
	else{
		while(!scan_rotation_ok && ros::ok()){
			if(!scan_rotation_ok){
				std::cout << "rotation_return(debag)なので速度送信" << std::endl;
				vel_pub.publish(vel);
			}
		
			scan_branch_queue.callOne(ros::WallDuration(1));//scan_angleに設定した角度の範に空間ができるまで回転する
		}
		std::cout << "rotation_true(debag)だったので終わり" << std::endl;
		scan_rotation_ok = false;
	}
}


void VFH_gravity(const sensor_msgs::LaserScan::ConstPtr& scan_msg){//引力の影響を受けた目標角度を決める
	vfh_gra_x = goal_point_x - odom_x;
	vfh_gra_y = goal_point_y - odom_y;
	float goal_angle_v;

	display_gravity(goal_point_x, goal_point_y);

	gra_angle = atan2(vfh_gra_y,vfh_gra_x);//ロボットの現在座標から目標に対してのベクトル座標//-180~180

//ここからセンサベースと同じVFH/////////////////////////////////////////////////////

	const float angle_min = scan_msg->angle_min;
	const float angle_max = scan_msg->angle_max;
	const float angle_increment = scan_msg->angle_increment;
	const float all_nan = 3.14;

	std::vector<float> ranges = scan_msg->ranges;
	float rad;
	std::vector<float> angles;

	//スキャンデータを極座標から直交座標に直すやつ///
	for(int i=0;i<ranges.size();i++){
		rad = angle_min+(angle_increment*i);
		ranges[i] = ranges[i] * cos(rad);
		angles.push_back(rad);
	}
	/////////////////////

	approx(ranges);

	goal_angle_v = VFH_move_angle_g(ranges,angle_min,angle_increment,all_nan,gra_angle,angles);

	bumper_queue.callOne(ros::WallDuration(1));

	if(goal_angle_v >= all_nan){
		vel_recovery_g();
	}
	else if(bumper_hit){
		vel_recovery_g();
	}

	else{
		need_back = true;
		need_rotate_calc = true;
		vel_curve_VFH(goal_angle_v, angle_max);	
	}


	//自分のヨー角をみてタンジェントの角が第何象限にあるかを考える
	//VFHで見つかった安全な角度の中から最も引力の向きに近い角度を目標角度に設定する

}


void VFH4vel_publish_Branch(){
	const float goal_margin = 0.7;
	bool finish_flag = false;
	float now2goal_dis;


	odom_queue.callOne(ros::WallDuration(1));//自分のオドメトリ取得	

	goal_point_x = odom_x + cos(yaw)*goal_x - sin(yaw)*goal_y;
	goal_point_y = odom_y + sin(yaw)*goal_x + cos(yaw)*goal_y;

	//goal_point_x = 1.7;
	//goal_point_y = -0.7;

	std::cout << "目標へ移動開始" << std::endl;
	std::cout << "goal(" << goal_point_x << "," << goal_point_y << ")" << std::endl;

	std::cout << "now(" << odom_x << "," << odom_y << ")\n" << std::endl;
	
	while(!finish_flag && ros::ok()){
		odom_marking(odom_x,odom_y);
		std::cout << "1" << std::endl;
		scan_queue.callOne(ros::WallDuration(1));//重力の影響を受けた進行方向を決めて速度を送る
		std::cout << "2" << std::endl;
		odom_queue.callOne(ros::WallDuration(1));//自分のオドメトリ取得
		std::cout << "goal(" << goal_point_x << "," << goal_point_y << ")" << std::endl;
		std::cout << "now(" << odom_x << "," << odom_y << ")\n" << std::endl;
		now2goal_dis = sqrt(pow(goal_point_x-odom_x,2)+pow(goal_point_y-odom_y,2));
		if(now2goal_dis < goal_margin){
			break;
		}
	}
	std::cout << "目標へ移動終了" << std::endl;
	branch_find_flag = false;
	display_gravity(odom_x, odom_y);


}

/*旧バージョン
void VFH4vel_publish_Branch(){
	double y;
	double y2;
	float start_x;
	float start_y;
	float odom_abs;
	double old_y2;
	bool change_sign = false;


	odom_queue.callOne(ros::WallDuration(1));

	std::cout << "(" << odom_x << "," << odom_y << ")" << std::endl;	

	start_x = odom_x;
	start_y = odom_y;
	y = yaw;

	std::cout << "分岐地点にいっくよ〜〜!" << std::endl;

	branch_forward(start_x,start_y,goal_x);

	std::cout << "ふぅ〜、やっと着いた" << std::endl;

	std::cout << "それじゃあ分岐方向に回転するね!" << std::endl;


	//branch_rotate_odom(y,goal_y,right_angle);//回転終了をオドメトリから決める
	branch_rotate_scan();//回転終了をスキャンから決める


    	branch_find_flag = false;
	after_rotate = true;
	pre_rotate = vel.angular.z;
	

	std::cout << "到着した分岐領域の座標 (" << odom_x << "," << odom_y << ")" << std::endl;


	std::cout << "回転できた!!ぜんし〜ん" << std::endl;
}
*/
void Branch_area_callback(const sensor_msgs::LaserScan::ConstPtr& Branch_msg){
	const float angle_min = Branch_msg->angle_min;
	const float angle_increment = Branch_msg->angle_increment;


	std::vector<float> ranges = Branch_msg->ranges;
	std::vector<float> fixed_ranges;
	std::vector<float> fixed_angle;

	//分岐領域でも線形近似してみる///それに伴って分岐閾値変更1.5->1.0//悪影響かも
	//approx(ranges);

	int scan_width = branch_angle/angle_increment;//99

	int scan_i_min = (ranges.size()/2)-1 - scan_width;
	int scan_i_max = (ranges.size()/2) + scan_width;

	for(int i = scan_i_min;i<scan_i_max;i++){
		if(!isnan(ranges[i]) && ranges[i] < branch_obst_limit){
			return;
		}
	}



	/*
	if(!isnan(ranges[ranges.size()/2]) && ranges[ranges.size()/2] < branch_obst_limit){
		return;
	} 
	*/

	for(int i=0;i<ranges.size();i++){
		if(!isnan(ranges[i])){
			fixed_ranges.push_back(ranges[i]);
			fixed_angle.push_back(angle_min+(angle_increment*i));
		}
	}

	if(fixed_ranges.size() < 2){
		//std::cout << "skip(debag)" << std::endl;
		return;
	}

	std::cout << "分岐領域どこだろ〜" << std::endl;
	std::cout << "・・・・・・" << std::endl;

	Branch_search(fixed_ranges,fixed_angle);

	std::cout << "・・・・・・" << std::endl;		
	
	if(branch_find_flag){
		std::cout << "みーつけた!!" << std::endl;
		std::cout << "*****角度 " << goal_angle*180/3.141592 << " [deg] " << " 座標 (" << goal_x << "," << goal_y << ")*****" << std::endl;
		duplicated_point_detection();

		if(branch_find_flag){
			VFH4vel_publish_Branch();
		}
	}
	else{
		std::cout << "う〜ん、無いな〜" << std::endl;
	}
}

void VFH_scan_callback(const sensor_msgs::LaserScan::ConstPtr& VFH_msg){
	const float angle_min = VFH_msg->angle_min;
	const float angle_increment = VFH_msg->angle_increment;
	const float all_nan = 3.14;

	std::vector<float> ranges = VFH_msg->ranges;
	float rad;
	float m_angle;
	float ang_g;
	float over_rad;
	float x_g;
	float y_g;
	std::vector<float> angles;




	//スキャンデータを極座標から直交座標に直すやつ///
	for(int i=0;i<ranges.size();i++){
		rad = angle_min+(angle_increment*i);
		ranges[i] = ranges[i] * cos(rad);
		angles.push_back(rad);
	}
	/////////////////////

	approx(ranges);

	m_angle = VFH_move_angle(ranges,angle_min,angle_increment,all_nan,angles);

	if(m_angle >= all_nan){
		if(after_rotate){
			vel_after_rotate();
			after_rotate = false;
		}
		else{
			vel_recovery();
		}
	}
	else{
		need_back = true;
		need_rotate_calc = true;
		after_rotate = false;


		odom_queue.callOne(ros::WallDuration(1));
		
		ang_g = yaw + m_angle;
		if(ang_g > PI){
			over_rad = ang_g - PI;
			ang_g = -PI + over_rad;
		}
		if(ang_g < -PI){
			over_rad = ang_g + PI;
			ang_g = PI - over_rad;
		}

		x_g = scan_threshold * cos(ang_g) + odom_x;
		y_g = scan_threshold * sin(ang_g) + odom_y;

		display_goal_angle(x_g, y_g);	


		vel_curve_VFH(m_angle,-angle_min);
		odom_queue.callOne(ros::WallDuration(1));
		odom_marking(odom_x,odom_y);	
	}
}


int main(int argc, char** argv){
	ros::init(argc, argv, "new_sensor_based_exploration");
	ros::NodeHandle s;

	VFH_option = ros::SubscribeOptions::create<sensor_msgs::LaserScan>("/scan",1,VFH_scan_callback,ros::VoidPtr(),&VFH_queue);
	VFH_sub = s.subscribe(VFH_option);

	Branch_option = ros::SubscribeOptions::create<sensor_msgs::LaserScan>("/scan",1,Branch_area_callback,ros::VoidPtr(),&Branch_queue);
	Branch_sub = s.subscribe(Branch_option);

	odom_option = ros::SubscribeOptions::create<geometry_msgs::Point>("/odom_support",1,odom_callback,ros::VoidPtr(),&odom_queue);
	odom_sub = s.subscribe(odom_option);

	scan_branch_option = ros::SubscribeOptions::create<sensor_msgs::LaserScan>("/scan",1,scan_branch_callback,ros::VoidPtr(),&scan_branch_queue);
	scan_branch_sub = s.subscribe(scan_branch_option);


	tf_option = ros::SubscribeOptions::create<tf2_msgs::TFMessage>("/tf",1,tf_callback,ros::VoidPtr(),&tf_queue);
	tf_sub = s.subscribe(tf_option);

	scan_option = ros::SubscribeOptions::create<sensor_msgs::LaserScan>("/scan",1,VFH_gravity,ros::VoidPtr(),&scan_queue);
	scan_sub = s.subscribe(scan_option);

	bumper_option = ros::SubscribeOptions::create<kobuki_msgs::BumperEvent>("/bumper_info",1,bumper,ros::VoidPtr(),&bumper_queue);
	bumper_sub = s.subscribe(bumper_option);


	vel_pub = s.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
	marker_pub = s.advertise<visualization_msgs::Marker>("visualization_marker", 1);

	vel.linear.y = 0;
	vel.linear.z = 0;
	vel.angular.x = 0;
	vel.angular.y = 0;

	marker3.header.frame_id = "map";
	marker3.header.stamp = ros::Time::now();
	marker3.ns = "basic_shapes";
	marker3.id = 2;
	marker3.type = list;
	marker3.action = visualization_msgs::Marker::ADD;
	marker3.lifetime = ros::Duration(0);
	marker3.pose.orientation.w = 1.0;
	marker3.scale.x = 0.1;
	marker3.color.b = 0.0f;
	marker3.color.a = 1.0;
	marker3.color.r = 1.0f;
	marker3.color.g = 1.0f;


	if(AI_wakeup){
		AI_Wakeup_Performance();
	}

	while(ros::ok()){
		if(need_back){
			Branch_queue.callOne(ros::WallDuration(1));
		}
		VFH_queue.callOne(ros::WallDuration(1));
		tf_queue.callOne(ros::WallDuration(1));
		if(loop_count == loop_closing_max){
			std::cout << "ループクロージングを" << loop_closing_max << "回したので終了" << std::endl;
			break;
		}
		std::cout << "loop_count : " << loop_count << std::endl;
	}



	return 0;

}
