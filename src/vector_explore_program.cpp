#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>//地図データ取得用
#include <tf/transform_listener.h>//自己位置取得用
#include <move_base_msgs/MoveBaseAction.h>//ナビゲーション用
#include <actionlib/client/simple_action_client.h>//ナビゲーション用
#include <geometry_msgs/Twist.h>//回転速度送信用
#include <fstream>//ファイル出力用
#include <sys/stat.h>//ディレクトリ作成用
#include <std_msgs/Bool.h>

//グローバル変数////////////////////////////////////////////////////////////////////////////////////////////
ros::Publisher vel_pub;
ros::Subscriber map_sub;

ros::Publisher swi_pub;
ros::Subscriber switch_sub;
bool switcher=false;

bool stop = false;
float search_radius=100000.0;//スタート地点からの探査範囲制限(半径m)
move_base_msgs::MoveBaseGoal goal;
ros::Time start;//探査をスタートした時間
ros::Time process;//経過時間観測用
ros::Time now;
ros::Duration end;
//tf::TransformListener listener;
//tf::StampedTransform transform;
double end_sec;//処理時間計算用

double cycle_time;
double search_time;
double local_time;
double local_prog;
double select_time;
double select_prog;
double trans_time;
double trans_prog;
double move_time;
double move_prog;
float pre_vector_x = 0;
float pre_vector_y = 0;

bool first_cycle = true;

//bool first_calc = true;

void robot_rotate();


/*
void export_data(int square, double cycle, double search, double local, double select, double trans, double move){
	std::ofstream ofs("./experimental_data/Processing_time.csv",std::ios::app);
	ofs << square << "," << cycle << "," << search << "," << local << "," << select << "," << trans << "," << move << std::endl;
}

void debag_data1(int point_num,int i,float ro_x_map,float ro_y_map,float fro_x_tmp,float fro_y_tmp,float pre_vector_x,float pre_vector_y,float dis_tmp,float dot_tmp){
	std::ofstream data1("./debag_data1/debag1.csv",std::ios::app);
	data1 << point_num << "," << i << "," << ro_x_map << "," << ro_y_map << "," << fro_x_tmp << "," << fro_y_tmp << "," << pre_vector_x  << "," << pre_vector_y << "," << dis_tmp << "," << dot_tmp << std::endl;
}

void debag_data2(int i,int goal_num,float dot_max,float length_max,float EVA,float EVA_max){
	std::ofstream data2("./debag_data2/debag2.csv",std::ios::app);
	data2 << i << "," << goal_num << "," << dot_max << "," << length_max << "," << EVA << "," << EVA_max << std::endl;
}

void debag_data3(int goal_num,float fro_x_tmp,float fro_y_tmp,float EVA_max,float pre_vector_x,float pre_vector_y){
	std::ofstream data3("./debag_data3/debag3.csv",std::ios::app);
	data3 << goal_num << "," << fro_x_tmp << "," << fro_y_tmp << "," << EVA_max << "," << pre_vector_x << "," << pre_vector_y <<  std::endl;
}
*/
//目標座標にナビゲーションする関数/////////////////////////////////////////////////////////////////////////////
bool navigation(float far_x,float far_y){

	process = ros::Time::now();

	//std::cout << "＊＊＊＊＊＊＊＊＊＊目標座標を受信＊＊＊＊＊＊＊＊＊＊" << std::endl;

	//define a client for to send goal requests to the move_base server through a SimpleActionClient

	//actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("robot1/move_base", true);
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

	/*actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", false);
	ros::spinOnce();
	*/
	//wait for the action server to come up//5.0秒まで待つ
	while(!ac.waitForServer(ros::Duration(5.0)) && ros::ok()){
		std::cout << "＊＊＊＊＊＊＊＊＊＊待機中＊＊＊＊＊＊＊＊＊＊" << std::endl;;
	}

	//move_base_msgs::MoveBaseGoal goal;

	//set up the frame parameters
	//goal.target_pose.header.frame_id = "robot1_tf/map";
	/*goal.target_pose.header.frame_id = "map";

	goal.target_pose.header.stamp = ros::Time::now();*/

	/* moving towards the goal*/

	goal.target_pose.pose.position.x =  far_x;
	goal.target_pose.pose.position.y =  far_y;
	goal.target_pose.pose.position.z =  0.0;
	goal.target_pose.pose.orientation.x = 0.0;
	goal.target_pose.pose.orientation.y = 0.0;
	goal.target_pose.pose.orientation.z = 0.0;
	goal.target_pose.pose.orientation.w = 1.0;

	end = ros::Time::now() - start;
	end_sec = end.toSec();

	std::cout << "＊＊＊＊＊＊＊＊＊＊目標座標セット(" << far_x << "," << far_y << ")＊＊＊＊＊＊＊＊＊＊" << " ここまで" << end_sec << "[s]" << std::endl;

	end = ros::Time::now() - process;
	trans_prog = end.toSec();
	trans_time += trans_prog;

	process = ros::Time::now();

	std::cout << "＊＊＊＊＊＊＊＊＊＊経路を作成中＊＊＊＊＊＊＊＊＊＊" << std::endl;
	ac.sendGoal(goal);
	std::cout << "＊＊＊＊＊＊＊＊＊＊sendend＊＊＊＊＊＊＊＊＊＊" << std::endl;
	ac.waitForResult();
	std::cout << "＊＊＊＊＊＊＊＊＊＊waitend＊＊＊＊＊＊＊＊＊＊" << std::endl;
	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
		end = ros::Time::now() - start;
		end_sec = end.toSec();
		std::cout << "＊＊＊＊＊＊＊＊＊＊目標座標に到着＊＊＊＊＊＊＊＊＊＊" << " ここまで" << end_sec << "[s]" << std::endl;
		return  true;
	}
	else{
		end = ros::Time::now() - start;
		end_sec = end.toSec();
		std::cout << "＊＊＊＊＊＊＊＊＊＊目標座標への移動不可＊＊＊＊＊＊＊＊＊＊" << " ここまで" << end_sec << "[s]" << std::endl;
		return  false;
	}
}

//現在位置に対して制限範囲の中で最も近い領域を探す関数////////////////////////////////////////////////////////////

void far_frontier(std::vector<float> fro_x, std::vector<float> fro_y, int fro_num){

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
	int retry_end = 4;

	bool limit_fro;
	float distance;
		

	std::cout << "start:far_frontier"  << std::endl;
	
	std::cout << "＊＊＊＊＊＊＊＊＊＊現在の探査半径 (" << search_radius << " m )＊＊＊＊＊＊＊＊＊＊" << std::endl;
	
research://再検索のときに帰ってくる場所
	process = ros::Time::now();

	limit_fro =true;
	distance=100;

	tf::TransformListener listener;
    	tf::StampedTransform transform;

	

	if(fro_num_tmp == 0){
		std::cout << "未探査座標が無いからskipしたよ"  << std::endl;
		goto skip;
	}

	end = ros::Time::now() - start;
	end_sec = end.toSec();
	std::cout << "start:ロボットの現在座標を取得" << " ここまで" << end_sec << "[s]" << std::endl;

	//tf::TransformListener listener;
    	//tf::StampedTransform transform;

     	now = ros::Time::now();
      	//listener.waitForTransform("/robot1_tf/map", "/robot1_tf/base_footprint", now, ros::Duration(1.0));
      	//listener.lookupTransform("/robot1_tf/map", "/robot1_tf/base_footprint",ros::Time(0), transform);
      	listener.waitForTransform("/map", "/base_footprint", now, ros::Duration(1.0));
      	listener.lookupTransform("/map", "/base_footprint",ros::Time(0), transform);

	ro_x_map = transform.getOrigin().x();
	ro_y_map = transform.getOrigin().y();

	std::cout << "現在座標 (" << ro_x_map << "," << ro_y_map <<  ")" <<std::endl;

	end = ros::Time::now() - start;
	end_sec = end.toSec();
	std::cout << "end  :ロボットの現在座標を取得" << " ここまで" << end_sec << "[s]" << std::endl;
	
	end = ros::Time::now() - process;
	local_prog = end.toSec();
	local_time += local_prog; 

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
		
	process = ros::Time::now();

	point_num = 0;
	first_calc = true;

	for(i=0;i<fro_num_tmp;i++){
		dis_rad = sqrt(pow(fro_x_tmp[i], 2) + pow(fro_y_tmp[i],2));
		if(dis_rad <= search_radius){
			x_tmp = fro_x_tmp[i] - ro_x_map;
			y_tmp = fro_y_tmp[i] - ro_y_map;
			dis_tmp = sqrt(pow(x_tmp, 2) + pow(y_tmp,2));
			dot_tmp = (x_tmp*pre_vector_x + y_tmp*pre_vector_y)/dis_tmp;
			dot.push_back(dot_tmp);
			length.push_back(dis_tmp);
			i_list.push_back(i);

			//debag_data1(point_num,i,ro_x_map,ro_y_map,fro_x_tmp[i],fro_y_tmp[i],pre_vector_x,pre_vector_y,dis_tmp,dot_tmp);
			//std::cout << "data1_export" << std::endl;
			
			point_num++;
			limit_fro = false;
	
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
	}

skip:

	//制限範囲内に未探査領域がなくなったら範囲を広げる////////////////////////////////////////////////////////
	if(limit_fro){
		if(search_radius == 30 ){
			stop = true;
			goto far_end;		
		}
		std::cout << "＊＊＊＊＊＊＊＊＊＊制限範囲内に未探査領域なし＊＊＊＊＊＊＊＊＊＊" << std::endl;
		search_radius += 2.0;
		std::cout << "＊＊＊＊＊＊＊＊＊＊探査半径を+2m ( 現在" << search_radius << "m ) ＊＊＊＊＊＊＊＊＊＊" << std::endl;
		end = ros::Time::now() - start;
		end_sec = end.toSec();
		std::cout << "end  :制限範囲内にある座標に対して距離とベクトルの内積を計算し目標を決定" << " ここまで" << end_sec << "[s]" << std::endl;
		goto far_end;
	}
	

	

	//保存するデータ
	//ポイント番号,番号、現在x,現在y、未探査x、未探査y,以前のベクトルx、以前のベクトルy、距離、内積
	//point_num,i,ro_x_map,ro_y_map,fro_x_tmp[i],fro_y_tmp[i],pre_vector_x,pre_vector_y,dis_tmp,dot_tmp
	
	
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

		//debag_data2(i,goal_num,dot_max,length_max,EVA,EVA_max);	
		//std::cout << "data2_export" << std::endl;	
		
	}
	//保存するデータ
	//i,goal_num,dot_max,length_max,EVA,EVA_max

	first_cycle = false;

	//pre_vector_x = fro_x_tmp[goal_num] - ro_x_map;
	//pre_vector_y = fro_y_tmp[goal_num] - ro_y_map;


	//debag_data3(goal_num,fro_x_tmp[goal_num],fro_y_tmp[goal_num],EVA_max,pre_vector_x,pre_vector_y);
	//std::cout << "data3_export" << std::endl;
	//保存するデータ
	//goal_num,fro_x_tmp[goal_num],fro_y_tmp[goal_num],EVA_max,pre_vector_x,pre_vector_y
	
	std::cout << "目標座標 (" << fro_x_tmp[goal_num] << "," << fro_y_tmp[goal_num] <<  ")" <<std::endl;

	//std::cout << "end  :制限範囲内にある座標に対して距離とベクトルの内積を計算し目標を決定" << std::endl;

	end = ros::Time::now() - start;
	end_sec = end.toSec();
	std::cout << "end  :制限範囲内にある座標に対して距離とベクトルの内積を計算し目標を決定" << " ここまで" << end_sec << "[s]" << std::endl;



//制限範囲内に未探査領域がなくなったら範囲を広げる////////////////////////////////////////////////////////
/*	if(limit_fro){
		if(search_radius == 30 ){
			stop = true;
			goto far_end;		
		}
		std::cout << "＊＊＊＊＊＊＊＊＊＊制限範囲内に未探査領域なし＊＊＊＊＊＊＊＊＊＊" << std::endl;
		search_radius += 2.0;
		std::cout << "＊＊＊＊＊＊＊＊＊＊探査半径を+2m ( 現在" << search_radius << "m ) ＊＊＊＊＊＊＊＊＊＊" << std::endl;
		goto far_end;
	}
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////
	std::cout << "＊＊＊＊＊＊＊＊＊＊現在の探査半径 (" << search_radius << " m )＊＊＊＊＊＊＊＊＊＊" << std::endl;
	std::cout << "現在座標 (" << ro_x_map << "," << ro_y_map <<  ")" <<std::endl;

	end = ros::Time::now() - process;
	select_prog = end.toSec();
	select_time += select_prog;

	//nav_success = navigation(far_x,far_y);//ナビゲション用の関数を呼び出す
	nav_success = navigation(fro_x_tmp[goal_num],fro_y_tmp[goal_num]);//ナビゲション用の関数を呼び出す


	if(nav_success == true){
		//robot_rotate();
		std::cout << "＊＊＊＊＊＊＊＊＊＊移動完了＊＊＊＊＊＊＊＊＊＊" << std::endl;
		
		pre_vector_x = fro_x_tmp[goal_num] - ro_x_map;
		pre_vector_y = fro_y_tmp[goal_num] - ro_y_map;
		
		end = ros::Time::now() - process;
		move_prog = end.toSec();
		move_time += move_prog;
	}

	else if(nav_success == false){
		//std::cout << "座標削除 (" << fro_x_tmp[num] << "," << fro_y_tmp[num] <<  ")　座標間距離" << distance << "m 　残り" << count-1 << "個" <<std::endl;
		std::cout << "座標削除 (" << fro_x_tmp[goal_num] << "," << fro_y_tmp[goal_num] << ")" <<std::endl;
		//fro_x_tmp.erase(fro_x_tmp.begin() + num);
		//fro_y_tmp.erase(fro_y_tmp.begin() + num);
		fro_x_tmp.erase(fro_x_tmp.begin() + goal_num);
		fro_y_tmp.erase(fro_y_tmp.begin() + goal_num);
		fro_num_tmp--;
	
		retry_counter++;
		
		if(retry_counter == retry_end){
			std::cout << "＊＊＊＊＊＊＊＊＊＊再検索回数オーバー＊＊＊＊＊＊＊＊＊＊" << std::endl;
			end = ros::Time::now() - process;
			move_prog = end.toSec();
			move_time += move_prog;
			goto far_end;
		}
		std::cout << "＊＊＊＊＊＊＊＊＊＊再検索 (" << retry_counter << "回目) スタート＊＊＊＊＊＊＊＊＊＊" << std::endl;
		end = ros::Time::now() - process;
		move_prog = end.toSec();
		move_time += move_prog;
		
		goto research;			
	}
	else{
		std::cout << "＊＊＊＊＊＊＊＊＊＊何かエラーがおきた＊＊＊＊＊＊＊＊＊＊" << std::endl;
		end = ros::Time::now() - process;
		move_prog = end.toSec();
		move_time += move_prog;
	}
	
far_end:
	end = ros::Time::now() - start;
	end_sec = end.toSec();
	std::cout << "end  :far_frontier" << " ここまで" << end_sec << "[s]" << std::endl;
///////////////////////////////////////////////////////////////////////////////////////////////////////////

}



//フロンティアを検索する関数//////////////////////////////////////////////////////////////////////////////////////////
void frontier_search(const nav_msgs::OccupancyGrid::ConstPtr& msg){

//地図データを配列に格納////////////////////////////////////////////////////////////////////////////////////////
	int fro_num;
	std::vector<float> fro_x;//見つけたフロンティアのx座標
	std::vector<float> fro_y;//見つけたフロンティアのy座標	
	nav_msgs::MapMetaData info = msg->info;//地図の設定を取得
	std::vector<int8_t> data = msg->data;//地図の値を取得
	int x = info.width;//地図の横サイズ
	int y = info.height;//地図の縦サイズ
	int square = x*y;//地図の面積
	int8_t map_array[x][y];//地図を行列に格納
	int frontier_flag[x][y];//探査済みと未探査の境界を判定するフラグ
	int point[x][y];//未探査領域の近くに障害物があるか判定する用
	int i,j;//for文
	int k = 0;//for文
	//int l = 0;//for文

	std::cout << "start:frontier_search" << std::endl;

	local_time = 0.0;
	select_time = 0.0;
	trans_time = 0.0;
	move_time = 0.0;
	
	start = ros::Time::now();//地図からの処理をスタートした時間

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
	end = ros::Time::now() - start;
	end_sec = end.toSec();
	std::cout << "end  :地図データを配列に格納" << " ここまで" << end_sec << "[s]" << std::endl;

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
	end = ros::Time::now() - start;
	end_sec = end.toSec();
	std::cout << "end  :横方向で境界を検索" << " ここまで" << end_sec << "[s]" << std::endl;

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
	end = ros::Time::now() - start;
	end_sec = end.toSec();
	std::cout << "end  :縦方向で境界を検索" << " ここまで" << end_sec << "[s]" << std::endl;

////////////////////////////////////////////////////////////////////////////////////////////////////////////






	
//横方向にフラグが8マス続いてる場所を探す////////////////////////////////////////////////////////////////////////
	float m_per_cell = info.resolution;//[m/cell]
	float search_len = 0.6; //障害物を検索する正方形の一辺の長さ[m](都合上10倍して自然数の偶数になる値のみで計算時は+m_per_cell[m]される)
	float robot_diameter = 0.4; //ロボットの直径は0.4[m]
	int search_len_cell = search_len / m_per_cell;//セル換算した正方形の一辺の長さ
	int robot_cellsize = robot_diameter / m_per_cell;//セル換算したロボットサイズ
	int frontier_sum;//フラグが続いているかの判定用
	float frontier_center;//フロンティア境界の中点
	float low_left_x = info.origin.position.x;//地図の左下のx座標
	float low_left_y = info.origin.position.y;//地図の左下のy座標
	int k_end;//検索の重複を防ぐ用
	int half_sq = search_len_cell / 2;//正方形の一辺の半分の長さ(セル)
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
	const int search_move = 1;
	const int search_margin = search_width/2;

	for(i=search_margin;i<(y-search_margin);i=i+search_move){
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


	for(j=search_margin;j<(x-search_margin);j=j+search_move){
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


/*
	std::cout << "start:横方向で境界が連続している場所を検索" << std::endl;





	//for(i=1;i<(y-1);i++){//一行ごとに検索
	for(i=1;i<(y-1);i=i+3){//三行ごとに検索
		//old_search////////////////////////////////////////////////////////////////
		for(j=0;j<(x-robot_cellsize);j=j+3){//三列ごとに検索
    		//for(j=0;j<(x-robot_cellsize);j++){//一列ごとに検索
			frontier_sum = 0;
			
			for(k=j;k<(j+robot_cellsize);k++){
				frontier_sum=frontier_sum+frontier_flag[k][i];
				if(frontier_flag[k][i] == 0 && ((frontier_flag[k][i-1] == 1) || (frontier_flag[k][i+1] == 1))){
					frontier_sum++;
				}
			}
			if(frontier_sum == robot_cellsize){
				frontier_center = (j+robot_cellsize-1)-(robot_cellsize/2);
				flo2int = frontier_center;
				point[flo2int][i] = 1;
				pre_frox.push_back(flo2int);
				pre_froy.push_back(i);
				pre_fronum++;
			}
		}		
  	}
	end = ros::Time::now() - start;
	end_sec = end.toSec();
	std::cout << "end  :横方向で境界が連続している場所を検索" << " ここまで" << end_sec << "[s]" << std::endl;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////

//縦方向にフラグが8マス続いてる場所を探す////////////////////////////////////////////////////////////////////////

	std::cout << "start:縦方向で境界が連続している場所を検索" << std::endl;

	//for(j=1;j<(x-1);j++){//一列ごとに検索
	for(j=1;j<(x-1);j=j+3){//三列ごとに検索
		//old_search
		for(i=0;i<(y-robot_cellsize);i=i+3){//三列ごとに検索
    		//for(i=0;i<(y-robot_cellsize);i++){//一列ごとに検索
			frontier_sum = 0;
			for(k=i;k<(i+robot_cellsize);k++){
				frontier_sum=frontier_sum+frontier_flag[j][k];
				if(frontier_flag[j][k] == 0 && ((frontier_flag[j-1][k] == 1) || (frontier_flag[j+1][k] == 1))){
					frontier_sum++;
				}	
			}
			if(frontier_sum == robot_cellsize){
				frontier_center = (i+robot_cellsize -1)-(robot_cellsize/2);
				flo2int = frontier_center;
				point[j][flo2int] = 1;
				pre_frox.push_back(j);
				pre_froy.push_back(flo2int);
				pre_fronum++;
			}
    		}
  	}
	end = ros::Time::now() - start;
	end_sec = end.toSec();
	std::cout << "end  :縦方向で境界が連続している場所を検索" << " ここまで" << end_sec << "[s]" << std::endl;
*/
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
	end = ros::Time::now() - start;
	end_sec = end.toSec();
	std::cout << "end  :障害物情報を追加" << " ここまで" << end_sec << "[s]" << std::endl;
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

	end = ros::Time::now() - start;
	end_sec = end.toSec();
	std::cout << "end  :未探査領域周辺の障害物を検索" << " ここまで" << end_sec << "[s]" << std::endl;
	

	end = ros::Time::now() - start;
	search_time = end.toSec();
		
	if(fro_num == 0){
		stop = true;
		goto search_end;
	
	}

////////////////////////////////////////////////////////////////////

	far_frontier(fro_x, fro_y, fro_num);
search_end:
	if(stop){
		end = ros::Time::now() - start;
		end_sec = end.toSec();
		std::cout << "end  :探査プログラム" << " ここまで" << end_sec << "[s]" << std::endl;
		map_sub.shutdown();
	}

//処理時間の詳細を表示//////////////////////////////////////////////////////////////////////////////////////
	end = ros::Time::now() - start;
	cycle_time = end.toSec();

	std::cout << "\n処理時間＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊\n" << std::endl;
	//地図の面積
	std::cout << "地図のサイズ" << "\t\t\t" << square << "[pixel*pixel]" << std::endl;
	//移動完了までの時間
	std::cout << "サイクル全体" << "\t\t\t" << cycle_time << "[s]" << std::endl;
	//地図全体からの未探査領域検索時間
	std::cout << "未探査領域の検索" << "\t\t" << search_time << "[s]" << std::endl;
	//自己位置を取得する時間
	std::cout << "自己位置の取得" << "\t\t\t" << local_time << "[s]" << std::endl;
	//最も近い未探査領域の検索時間
	std::cout << "目標座標の選択" << "\t\t\t" << select_time << "[s]" << std::endl;
	//目標座標をROS_Navigationに送信する時間
	std::cout << "目標座標をNavigationに送信" << "\t" << trans_time << "[s]" << std::endl;
	//移動中の時間
	std::cout << "目標地点までの移動" << "\t\t" << move_time << "[s]" << std::endl;
	std::cout << "\n＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊\n" << std::endl;

	//export_data(square, cycle_time, search_time, local_time, select_time, trans_time, move_time);

	std::cout << "end  :frontier_search" << std::endl;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////	


//360[deg]回転////////////////////////////////////////////////////////////////////////////////////////////////
void robot_rotate(){
 	geometry_msgs::Twist vel;
	vel.angular.z = 0.5;
	ros::Duration timeout(16.8);	
	ros::Time start_time = ros::Time::now();
	
	while(ros::Time::now() - start_time < timeout && ros::ok()){
		ros::Rate rate(10.0);
		vel_pub.publish(vel);
		rate.sleep();
	}
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////

//スイッチ用の関数///////////////////////
void switching(const std_msgs::Bool::ConstPtr& msg){
	bool judge = false;
	judge = msg->data;
	
	if(judge){
		std::cout << "＊＊＊＊＊＊＊＊＊＊探査プログラム切り替え中＊＊＊＊＊＊＊＊＊＊" << std::endl;
		switcher = true;
		switch_sub.shutdown();
	}
}
////////////////////////////////////////


//メイン関数////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv){

  	ros::init(argc, argv, "vector_explore_program");
  	ros::NodeHandle nh ;

	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();


//matlabからの切り替え用///
	/*
	//スイッチ部分//////////////////////////
	swi_pub = nh.advertise<std_msgs::Bool>("mat_end", 1000);
	ros::Rate r(1);
	while(!switcher && ros::ok()){
		std::cout << "＊＊＊＊＊＊＊＊＊＊切り替え待機中＊＊＊＊＊＊＊＊＊＊" << std::endl;
		switch_sub = nh.subscribe("/switch",1,switching);
		ros::spinOnce();
		r.sleep();
	}
	std::cout << "＊＊＊＊＊＊＊＊＊＊探査プログラム切り替え完了＊＊＊＊＊＊＊＊＊＊" << std::endl;
	*/
	////////////////////////////////////////
////////////




/*	//現在時刻を取得///////////////////////////////////////////////////////////////
	time_t timer;
   	struct tm *local;
   	timer = time(NULL);

   	local = localtime(&timer);
	
	//std::cout << local->tm_year+1900 << "/" << local->tm_mon+1 << "/" << local->tm_mday << " " << local->tm_hour << ":" << local->tm_min << ":" << local->tm_sec << std::endl;
	///////////////////////////////////////////////////////////////////////////////

	mkdir("experimental_data", 0755);

	mkdir("debag_data1", 0755);
	mkdir("debag_data2", 0755);
	mkdir("debag_data3", 0755);

	std::ofstream ofs("./experimental_data/Processing_time.csv",std::ios::app);
	
	ofs << local->tm_year+1900 << "/" << local->tm_mon+1 << "/" << local->tm_mday << " " << local->tm_hour << ":" << local->tm_min << ":" << local->tm_sec << std::endl;

	ofs << "面積[pixel*pixel],サイクル[s],未探査領域の検索[s],自己位置の取得[s],目標の選択[s],目標の送信[s],目標に移動[s]" << std::endl;
	
//デバッグ用/////////
	std::ofstream data1("./debag_data1/debag1.csv",std::ios::app);
	data1 << "point_num,i,ro_x_map,ro_y_map,fro_x_tmp[i],fro_y_tmp[i],pre_vector_x,pre_vector_y,dis_tmp,dot_tmp" << std::endl;

	std::ofstream data2("./debag_data2/debag2.csv",std::ios::app);
	data2 << "i,goal_num,dot_max,length_max,EVA,EVA_max" << std::endl;

	std::ofstream data3("./debag_data3/debag3.csv",std::ios::app);
	data3 << "goal_num,fro_x_tmp,fro_y_tmp,EVA_max,pre_vector_x,pre_vector_y" <<  std::endl;
/////////////////////
*/	
	std::cout << "start:探査プログラム" << std::endl;

	vel_pub = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 10);

	/*std::cout << "start:360°回転" << std::endl;
	robot_rotate();
	std::cout << "end  :360°回転" << std::endl;
	*/

  	map_sub = nh.subscribe("/map",1,frontier_search);	
	ros::spin();

	//std::cout << "切り替えたよ" << std::endl;
	return 0;
}
