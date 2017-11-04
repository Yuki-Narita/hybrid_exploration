#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>//地図データ取得用
#include <tf/transform_listener.h>//自己位置取得用
#include <move_base_msgs/MoveBaseAction.h>//ナビゲーション用
#include <actionlib/client/simple_action_client.h>//ナビゲーション用
#include <geometry_msgs/Twist.h>//回転速度送信用
#include <fstream>//ファイル出力用
#include <sys/stat.h>//ディレクトリ作成用
#include <ros/callback_queue.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

//グローバル変数////////////////////////////////////////////////////////////////////////////////////////////
ros::Publisher vel_pub;
ros::Subscriber map_sub;
ros::Subscriber odom_sub;

ros::CallbackQueue map_queue;
ros::SubscribeOptions map_option;

ros::CallbackQueue odom_queue;
ros::SubscribeOptions odom_option;

float odom_x;//オドメトリx
float odom_y;//オドメトリy
double roll;//ロール角(使わない)
double pitch;//ピッチ角(使わない)
double yaw;//ヨー角

bool stop = false;

float pre_vector_x = 0;
float pre_vector_y = 0;

bool first_cycle = true;


/*
void create_path(float goal_ｘ, float goal_y){

}
*/


void odom_callback(const nav_msgs::Odometry::ConstPtr& odom_msg){
	float quat_x;
	float quat_y;
	float quat_z;
	float quat_w;

	odom_x = odom_msg->pose.pose.position.x;
	odom_y = odom_msg->pose.pose.position.y;
	quat_x = odom_msg->pose.pose.orientation.x;
	quat_y = odom_msg->pose.pose.orientation.y;
	quat_z = odom_msg->pose.pose.orientation.z;
	quat_w = odom_msg->pose.pose.orientation.w;

	tf::Quaternion q(quat_x, quat_y, quat_z, quat_w);
	tf::Matrix3x3 m(q);
	m.getRPY(roll, pitch, yaw);

	//std::cout << "odomcall(" << odom_x << "," << odom_y << ")" << std::endl;
	//std::cout << "odomcall Roll: " << roll << ", Pitch: " << pitch << ", Yaw: " << yaw << std::endl;
}


//現在位置に対して制限範囲の中で最も近い領域を探す関数////////////////////////////////////////////////////////////

void choose_goal_frontier(std::vector<float> fro_x, std::vector<float> fro_y, int fro_num){

//ロボットの現在座標を取得//////////////////////////////////////////////////////////////////////////////////////
	float ro_x_map;//ロボットの現在のx座標
	float ro_y_map;//ロボットの現在のy座標
	std::vector<float> fro_x_tmp = fro_x;
	std::vector<float> fro_y_tmp = fro_y;
	std::vector<float> length; //距離格納用の配列
	float dot_max;//計算した内積の最大値
	float dot_tmp;//内積を一時保存
	float length_max;//計算した距離の最大値
	std::vector<float> dot;//計算した内積を保存
	int fro_num_tmp = fro_num;
	int point_num;//制限範囲のフィルタをかけた後の個数
	std::vector<int> i_list;//制限範囲内にあったiの番号を保存
	float EVA;//評価式の計算結果
	float EVA_max;//評価式の最大値
	int goal_num;//目標地点の座標番号
	bool first_calc;
	float far_x;
	float far_y;
	float x_tmp;
	float y_tmp;
	float dis_tmp;
	int i;
	int num = 0;
	float dis_rad;
	bool nav_success;
	float dis_for_view;
	int retry_counter = 0;
	bool debag = false;

	float distance;
		

	std::cout << "start:far_frontier"  << std::endl;
	
	//std::cout << "＊＊＊＊＊＊＊＊＊＊現在の探査半径 (" << search_radius << " m )＊＊＊＊＊＊＊＊＊＊" << std::endl;

	distance=100;
	
	if(fro_num_tmp == 0){
		std::cout << "未探査座標が無いからskipしたよ"  << std::endl;
		goto skip;
	}

	std::cout << "start:ロボットの現在座標を取得" << std::endl;

	odom_queue.callOne(ros::WallDuration(1));

	ro_x_map = odom_x;
	ro_y_map = odom_y;

	std::cout << "現在座標 (" << ro_x_map << "," << ro_y_map <<  ")" <<std::endl;

	std::cout << "end  :ロボットの現在座標を取得" << std::endl;


//////////////////////////////////////////////////////////////////////////////////////////////////////////////

//現在位置から各目標に対してのベクトルを計算し、前回の移動と近い目標を探す///////////////////////////////////////////////
//先に距離でフィルタをかけてから各計算をする
//現在位置//ro_x_map,ro_y_map//
//目標座標のxy//fro_x_tmp,fro_y_tmp//
//ベクトル格納用の配列xy//vector_x,vector_y//
//目標座標群の個数//fro_num_tmp//
//ベクトルの大きさの最大値//vector_max//
//距離格納用の配列//length//
//制限範囲のフィルタをかけた後の個数//point_num//
//過去のベクトル//pre_vector_x;pre_vector_y;
//過去のベクトルと目標へのベクトルの内積用配列//dot//
//制限範囲内にあったiを保存//i_list//fro_x_tmp[i]を保存している

	std::cout << "start:制限範囲内にある座標に対して距離とベクトルの内積を計算し目標を決定" << std::endl;

	point_num = 0;
	first_calc = true;

	for(i=0;i<fro_num_tmp;i++){
		dis_rad = sqrt(pow(fro_x_tmp[i], 2) + pow(fro_y_tmp[i],2));
		//if(dis_rad <= search_radius){
			x_tmp = fro_x_tmp[i] - ro_x_map;
			y_tmp = fro_y_tmp[i] - ro_y_map;
			dis_tmp = sqrt(pow(x_tmp, 2) + pow(y_tmp,2));
			dot_tmp = (x_tmp*pre_vector_x + y_tmp*pre_vector_y)/dis_tmp;
			dot.push_back(dot_tmp);
			length.push_back(dis_tmp);
			i_list.push_back(i);
			
			point_num++;
	
			if(first_calc){
				dot_max = dot_tmp;
				length_max = dis_tmp;
				first_calc = false;
			}
			if(dot_tmp>dot_max){
				dot_max = dot_tmp;
			}
			if(dis_tmp>length_max){
				length_max = dis_tmp;
			}
	}

 skip:


	first_calc = true;

	for(i=0;i<point_num;i++){
		
		if(first_cycle){
			EVA = (-(length[i]/length_max));
		}
		else{
			EVA = ((dot[i]/dot_max)-(length[i]/length_max));
		}

		if(first_calc){
			EVA_max = EVA;
			goal_num = i_list[i];
			first_calc = false;
		}
		if(EVA>EVA_max){
			EVA_max = EVA;
			goal_num = i_list[i];
		}
	}

	first_cycle = false;

	
	std::cout << "目標座標 (" << fro_x_tmp[goal_num] << "," << fro_y_tmp[goal_num] <<  ")" <<std::endl;

	std::cout << "end  :制限範囲内にある座標に対して距離とベクトルの内積を計算し目標を決定" << std::endl;


	//std::cout << "＊＊＊＊＊＊＊＊＊＊現在の探査半径 (" << search_radius << " m )＊＊＊＊＊＊＊＊＊＊" << std::endl;
	std::cout << "現在座標 (" << ro_x_map << "," << ro_y_map <<  ")" <<std::endl;


////////////////////////*******//ここで経路作成関数に目標を渡す///******////////////////////////////////
//create_path(fro_x_tmp[goal_num], fro_y_tmp[goal_num])
////////////////////////////////////////////////////////////////////////////////////////////////////


	std::cout << "end  :far_frontier" << std::endl;
}



//フロンティアを検索する関数//////////////////////////////////////////////////////////////////////////////////////////
void frontier_search(const nav_msgs::OccupancyGrid::ConstPtr& msg){

//地図データを配列に格納////////////////////////////////////////////////////////////////////////////////////////
	const nav_msgs::MapMetaData info = msg->info;//地図の設定を取得
	const std::vector<int8_t> data = msg->data;//地図の値を取得
	const int x = info.width;//地図の横サイズ
	const int y = info.height;//地図の縦サイズ


	int fro_num;
	std::vector<float> fro_x;//見つけたフロンティアのx座標
	std::vector<float> fro_y;//見つけたフロンティアのy座標	
	int8_t map_array[x][y];//地図を行列に格納
	int frontier_flag[x][y];//探査済みと未探査の境界を判定するフラグ
	int point[x][y];//未探査領域の近くに障害物があるか判定する用
	int i,j;//for文
	int k = 0;//for文


	std::cout << "start:frontier_search" << std::endl;

	std::cout << "start:地図データを配列に格納" << std::endl;

	for(i=0;i<y;i++){
    		for(j=0;j<x;j++){
      			map_array[j][i] = data[k];
			if(map_array[j][i]!=0 && map_array[j][i]!=100 && map_array[j][i]!=-1){
					std::cout << "exception:" << map_array[j][i] << std::endl;		
			}
			frontier_flag[j][i] = 0;
			point[j][i] = 0;
      			k++;
    		}
  	}

	std::cout << "end  :地図データを配列に格納" << std::endl;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//横方向で未探査と探査済の境界を探す//////////////////////////////////////////////////////////////////////////////

	std::cout << "start:横方向で境界を検索" << std::endl;

	for(i=0;i<y;i++){
    		for(j=0;j<(x-1);j++){
      			if(map_array[j][i] == 0 && map_array[j+1][i] == -1){
	       			frontier_flag[j][i] = 1;
			}
        		else if(map_array[j][i] == -1 && map_array[j+1][i] == 0){
				frontier_flag[j+1][i] = 1;	
			}
    		}
  	}
	std::cout << "end  :横方向で境界を検索" << " ここまで" << std::endl;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
//縦方向で未探査と探査済の境界を探す/////////////////////////////////////////////////////////////////////////////

	std::cout << "start:縦方向で境界を検索" << std::endl;

	for(j=0;j<x;j++){
    		for(i=0;i<(y-1);i++){
      			if(map_array[j][i] == 0 && map_array[j][i+1] == -1){
	       			frontier_flag[j][i] = 1;
			}
        		else if(map_array[j][i] == -1 && map_array[j][i+1] == 0){
				frontier_flag[j][i+1] = 1;	
			}
    		}
  	}

	std::cout << "end  :縦方向で境界を検索" << std::endl;

////////////////////////////////////////////////////////////////////////////////////////////////////////////

	
//横方向にフラグが8マス続いてる場所を探す////////////////////////////////////////////////////////////////////////
	const float m_per_cell = info.resolution;//[m/cell]
	const float search_len = 0.4; //障害物を検索する正方形の一辺の長さ[m](都合上10倍して自然数の偶数になる値のみで計算時は+m_per_cell[m]される)
	const float robot_diameter = 0.4; //ロボットの直径は0.4[m]
	const int search_len_cell = search_len / m_per_cell;//セル換算した正方形の一辺の長さ
	const int robot_cellsize = robot_diameter / m_per_cell;//セル換算したロボットサイズ
	const float low_left_x = info.origin.position.x;//地図の左下のx座標
	const float low_left_y = info.origin.position.y;//地図の左下のy座標
	const int half_sq = search_len_cell / 2;//正方形の一辺の半分の長さ(セル)


	int frontier_sum;//フラグが続いているかの判定用
	float frontier_center;//フロンティア境界の中点
	int flo2int;
	std::vector<int> pre_frox;//未探査領域のx座標を一時保存
	std::vector<int> pre_froy;//未探査領域のy座標を一時保存
	int pre_fronum = 0;//未探査領域の個数を一時保存
	fro_num = 0;//フロンティアの個数を初期化


	std::cout << "start:横方向で境界が連続している場所を検索" << std::endl;
	int v;
	int continuity = 0;
	int start_k = 0;
	int end_k = 0;
	const int search_width = 3;//フラグの連続を検索するときの線の太さ(奇数)
	const int search_margin = search_width/2;

	for(i=search_margin;i<(y-search_margin);i=i+search_width){
		k = 0;
		while(k < x && ros::ok()){
			for(v=-search_margin;v<=search_margin;v++){
				frontier_sum += frontier_flag[k][i+v];
			}
			if(frontier_sum > 0){
				start_k = k;
				while(frontier_sum > 0 && ros::ok()){
					frontier_sum = 0;
					continuity++;
					if(k < x){
						k++;
						for(v=-search_margin;v<=search_margin;v++){
							frontier_sum += frontier_flag[k][i+v];
						}
					}
					else{	
						k++;
						break;
					}
					std::cout << k << "," << x << std::endl;
				}
				end_k = k-1;
				if(continuity >= robot_cellsize){
					frontier_center = (start_k + end_k)/2;
					flo2int = frontier_center;
					point[flo2int][i] = 1;
					pre_frox.push_back(flo2int);
					pre_froy.push_back(i);
					pre_fronum++;
				}
			}
			else{			
				k++;
			}
		}
	}
	
	std::cout << "end  :縦方向で境界が連続している場所を検索" << std::endl;


	for(j=search_margin;j<(x-search_margin);j=j+search_width){
		k = 0;
		while(k < y && ros::ok()){
			for(v=-search_margin;v<=search_margin;v++){
				frontier_sum += frontier_flag[j+v][k];
			}
			if(frontier_sum > 0){
				start_k = k;
				while(frontier_sum > 0 && ros::ok()){
					frontier_sum = 0;
					continuity++;
					if(k < y){
						k++;
						for(v=-search_margin;v<=search_margin;v++){
							frontier_sum += frontier_flag[j+v][k];
						}
					}
					else{	
						k++;
						break;
					}
				}
				end_k = k-1;
				if(continuity >= robot_cellsize){
					frontier_center = (start_k + end_k)/2;
					flo2int = frontier_center;
					point[j][flo2int] = 1;
					pre_frox.push_back(j);
					pre_froy.push_back(flo2int);
					pre_fronum++;
				}
			}
			else{			
				k++;
			}
		}
	}
	
	std::cout << "end  :縦方向で境界が連続している場所を検索" << std::endl;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//未探査領域配列に障害物情報を追加

	std::cout << "start:障害物情報を追加" << std::endl;

	for(j=0;j<x;j++){
    		for(i=0;i<y;i++){
      			if(map_array[j][i] == 100){
	       			point[j][i] = 100;
			}
    		}
  	}
	std::cout << "end  :障害物情報を追加" << std::endl;
//////////////////////////////////////////////////////////////////////////////////////////

//未探査領域の周囲の障害物情報を検索し最終的な未探査領域の座標を決定
	
//search_len_cell;//セル換算した正方形の一辺の長さ
//half_sq = search_len_cell / 2;//正方形の一辺の半分の長さ(セル)
//x;//地図の横サイズ
//y;//地図の縦サイズ
//pre_frox[];//未探査領域のx座標を一時保存
//pre_froy[];//未探査領域のy座標を一時保存
//pre_fronum = 0;//未探査領域の個数を一時保存
//point[x][y];//未探査領域と障害物情報がある行列
//frontier_sum;//未探査領域の中身の和

	int half_leftx;//四角形の左半分の長さ
	int half_rightx;//四角形の右半分の長さ
	int half_topy;//四角形の上半分の長さ
	int half_bottomy;//四角形の下半分の長さ
	
	std::cout << "start:未探査領域周辺の障害物を検索" << std::endl;

	for(k=0;k<pre_fronum;k++){
		
		if(pre_frox[k]-half_sq < 0){
			half_leftx = pre_frox[k];
		}
		else{
			half_leftx = half_sq;
		}


		if(pre_frox[k]+half_sq > (x-1)){
			half_rightx = (x-1)-pre_frox[k];
		}
		else{
			half_rightx = half_sq;
		}


		if(pre_froy[k]-half_sq < 0){
			half_topy = pre_froy[k];
		}
		else{
			half_topy = half_sq;
		}


		if(pre_froy[k]+half_sq > (y-1)){
			half_bottomy = (y-1)-pre_froy[k];
		}
		else{
			half_bottomy = half_sq;
		}
		
		frontier_sum = 0;
		
		for(i=(pre_froy[k]-half_topy);i<(pre_froy[k]+half_bottomy+1);i++){
			for(j=(pre_frox[k]-half_leftx);j<(pre_frox[k]+half_rightx+1);j++){
				frontier_sum+=point[j][i];
			}
		}

		if(frontier_sum>100){
			point[pre_frox[k]][pre_froy[k]] = 0;
		}
	}


	//最終的な未探査領域を配列に格納
	for(j=0;j<x;j++){
    		for(i=0;i<y;i++){
      			if(point[j][i] == 1){
	       			fro_x.push_back(m_per_cell * j + low_left_x);
				fro_y.push_back(m_per_cell * i + low_left_y);
				fro_num++;
			}
    		}
  	}

	std::cout << "障害物判定前の未探査領域の個数 " << pre_fronum << std::endl;
	std::cout << "障害物判定後の未探査領域の個数 " << fro_num << std::endl;

	std::cout << "end  :未探査領域周辺の障害物を検索" << std::endl;

	if(fro_num == 0){
		stop = true;
		return;
	}

	choose_goal_frontier(fro_x, fro_y, fro_num);

	std::cout << "end  :frontier_search" << std::endl;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////	


//メイン関数////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv){

  	ros::init(argc, argv, "new_vector_explore_program");
  	ros::NodeHandle f;


	map_option = ros::SubscribeOptions::create<nav_msgs::OccupancyGrid>("/map",1,frontier_search,ros::VoidPtr(),&map_queue);
	map_sub = f.subscribe(map_option);

	odom_option = ros::SubscribeOptions::create<nav_msgs::Odometry>("/odom",1,odom_callback,ros::VoidPtr(),&odom_queue);
	odom_sub = f.subscribe(odom_option);
	
	std::cout << "start:探査プログラム" << std::endl;

	vel_pub = f.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 10);

	while(!stop && ros::ok()){
		map_queue.callOne(ros::WallDuration(1));
	}
	std::cout << "end:探査プログラム" << std::endl;
	return 0;
}
