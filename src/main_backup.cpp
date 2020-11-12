#define FULL_TEST_DEBUG
#include "ros/ros.h"
#include "circle_find.cpp"
#include "img_tf.h"
#include <tf2_ros/transform_listener.h>
#include <tf/transform_listener.h>
#include "assembly_robot_msgs/cam_Srv.h"
#include <string>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <queue>
#include <algorithm>
#ifdef TEST_DEBUG
void op_handler(const std_msgs::Float32 &msg);
CircleFind *finder_1;

int main(int argc, char** argv){
	ros::init(argc, argv, "hole_finder_1");
	finder_1 = new CircleFind();
	ros::NodeHandle nh;
	ros::Subscriber op_sub = nh.subscribe("/camera_op",100,op_handler);
	ROS_INFO("READY CAMERA");
	ros::spin();
	delete finder_1;
	return 0;
}

void op_handler(const std_msgs::Float32 &msg){
	double output[3] {0,0,0};
	double input[3];
	finder_1->find_circle_tf(output,input);
}

#endif



#ifdef FULL_TEST_DEBUG
bool operate_1(assembly_robot_msgs::cam_Srv::Request &req,assembly_robot_msgs::cam_Srv::Response &res);
bool operate_2(assembly_robot_msgs::cam_Srv::Request &req,assembly_robot_msgs::cam_Srv::Response &res);

void find_adap_hole();
CircleFind *finder_1;
CircleFind *finder_2;
ImgTfFinder::CameraInfo cam1[2]{ {931.3824090000001,560.6960309999999,1460.136731,1459.304695}
    , {937.655922, 558.151482, 1475.831374,1473.475273}};
ImgTfFinder::CameraInfo cam2[2]{ {1014.006763,526.753957,1467.924445,1465.315384}
    , {959.665281, 593.683847, 1469.472732,1467.230973}};
tf2_ros::Buffer tfBuffer;
tf2_ros::TransformListener *tfListener;
int main(int argc, char** argv){
	ros::init(argc, argv, "hole_finder");
	ros::NodeHandle nh;
	string topic_1[2] = { "/camera1/usb_cam1_1/image_raw","/camera1/usb_cam1_2/image_raw"};
	string topic_2[2] = { "/camera2/usb_cam2_1/image_raw","/camera2/usb_cam2_2/image_raw"};
	finder_1 = new CircleFind(topic_1,cam1);
	finder_2 = new CircleFind(topic_2,cam2);
	tfListener = new tf2_ros::TransformListener(tfBuffer);
	ros::ServiceServer cam_server_1 = nh.advertiseService("camera_server_1",operate_1);
	ros::ServiceServer cam_server_2 = nh.advertiseService("camera_server_2",operate_2);
	ROS_INFO("READY CAMERA SERVER");
	ros::spin();	
	delete finder_1,finder_2,tfListener;
	return 0;
}
pair<pair<int,int>,pair<int,int>> reverse_to_pixel(double x,double y, double z,ImgTfFinder::CameraInfo cam[2]){
	x *= 1000;
	y *= 1000;
	z *= 1000;
	double d1 = z;
	double d2 = z;
	double Ux1 = (x+30)/z;
	double Ux2 = Ux1 - 2*30/z;
	double u1 = Ux1*cam[0].F_x + cam[0].C_x;
	double u2 = Ux2*cam[1].F_x + cam[1].C_x;
	double Uy = y/z;
	double v1 = cam[0].C_y- Uy*cam[0].F_y;
	double v2 = cam[0].C_y- Uy*cam[0].F_y;
	return pair<pair<int,int>,pair<int,int>>(make_pair(u1,v1),make_pair(u2,v2));
}
bool operate_1(assembly_robot_msgs::cam_Srv::Request &req,assembly_robot_msgs::cam_Srv::Response &res){
	using namespace std;
	string number[10] ={"1","2","3","4","5","6","7","8","9","10"}; 
	cout<<"RECIVED_DATA = "<<req.name<<endl;
	geometry_msgs::TransformStamped transform,hole_tf;
	queue<geometry_msgs::TransformStamped> tf_q;
	vector<pair<double,double>> tf_list;
	string hole_name = req.name;
	string part_num = hole_name.substr(0,6);
	vector<pair<int,int>> left_pair,right_pair;
	try{  
		hole_tf = tfBuffer.lookupTransform("camera_center_1",hole_name,ros::Time(0));
		for(int i=0;i != 10;i++){
			transform = tfBuffer.lookupTransform("camera_center_1",part_num+number[i],ros::Time(0));
			cout<<part_num+number[i]<<endl;
			tf_q.push(transform);
		}
	}
	catch (tf::TransformException ex){
		if(tf_q.empty()){
			ROS_ERROR("%s",ex.what());
			return false;
		}
		else
			ROS_INFO("FOUND %d HOLSE",(int)tf_q.size());
	}
	int q_size = tf_q.size();
	for(int i=0;i<q_size;i++){
		geometry_msgs::TransformStamped temp = tf_q.front();
		tf_q.pop();
		cout<<abs(temp.transform.translation.x - hole_tf.transform.translation.x)<<"\t"<<abs(temp.transform.translation.y - hole_tf.transform.translation.y)<<"\t"<<abs(temp.transform.translation.z - hole_tf.transform.translation.z)<<endl;
			if(abs(temp.transform.translation.x - hole_tf.transform.translation.x)<0.1 && abs(temp.transform.translation.y - hole_tf.transform.translation.y)<0.1 && abs(temp.transform.translation.z - hole_tf.transform.translation.z)<0.03){
				pair<pair<int,int>,pair<int,int>> pair_temp;
				cout<<"hole"<<i+1<<endl;
				cout<<temp.transform.translation.x<<" "<<temp.transform.translation.y<<endl;
				tf::Quaternion quat;
    			tf::quaternionMsgToTF(temp.transform.rotation, quat);
    			double roll, pitch, yaw;
    			tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
				cout<<roll<<" "<<pitch<<" "<<yaw<<endl;
				pair_temp = reverse_to_pixel(-temp.transform.translation.x,-temp.transform.translation.y,temp.transform.translation.z,cam1);
				left_pair.push_back(pair_temp.second);
				right_pair.push_back(pair_temp.first);
				cout<<endl<<pair_temp.second.first<<" "<<pair_temp.second.second<<" "<<pair_temp.first.first<<" "<<pair_temp.first.second<<endl<<endl;
				tf_list.push_back(make_pair(temp.transform.translation.x,temp.transform.translation.y));
			}
	}
	sort(tf_list.begin(),tf_list.end());
	sort(left_pair.begin(),left_pair.end());
	sort(right_pair.begin(),right_pair.end());
	for(int i=0;i<tf_list.size();i++){
		cout<<tf_list[i].first<<" "<<tf_list[i].second<<endl;
	}
	int to_find_index=tf_list.size() -1;
	for(int i=0;i<tf_list.size();i++){
		if(tf_list[i].first == hole_tf.transform.translation.x && tf_list[i].second==hole_tf.transform.translation.y)
			break;
		cout<< tf_list[i].first << " " << hole_tf.transform.translation.x << " " << tf_list[i].second << " "<<hole_tf.transform.translation.y<<"   ???"<<endl;
		to_find_index--;
	}
	double output[3] {0,0,0};
	cout<<"======================================================"<<endl;
	cout<<"====================== SERVICE START=================="<<endl<<endl;
	cout<<"HOLE NUM : "<<tf_list.size()<<endl;
	cout<<"FIND INDEX : "<<to_find_index<<endl;
	cout<<"============ FIND ==============="<<endl;
	res.result = finder_1->find_circle_tf(output,tf_list.size(),to_find_index,left_pair,right_pair);
	cout<<"============ RESULT ============="<<endl<<endl;
	cout<<"INSERT? : "<<(int)res.result<<endl;
	res.name = "camera_center_1";
	res.x = output[0]/1000;
	res.y = output[1]/1000;
	res.z = -output[2]/1000;
	cout<<"POSITNON \t : [ "<<res.x<<" , "<<res.y<<" , "<<res.z<<"]"<<endl<<endl;
	
	if(!res.result && !output[0]&& !output[1]&& !output[2])
		return false;
	if(res.result){
		geometry_msgs::TransformStamped transformStamped;
		tf2_ros::StaticTransformBroadcaster br;
		transformStamped.header.stamp = ros::Time::now();
		transformStamped.header.frame_id = "camera_center_1";
		transformStamped.child_frame_id = hole_name+"_target";
		transformStamped.transform.translation.x = res.x;
		transformStamped.transform.translation.y = res.y;
		transformStamped.transform.translation.z = res.z;
		transformStamped.transform.rotation.x = 0;
		transformStamped.transform.rotation.y = 0;
		transformStamped.transform.rotation.z = 0;
		transformStamped.transform.rotation.w = 1;
		int count =0;
		//while(count<5){
			//ros::Duration(0.2).sleep();
			br.sendTransform(transformStamped);
			count++;
			ros::Duration(0.2).sleep();
		//}
		cout<<"STAMP : "<<transformStamped.header.stamp<<endl;
		cout<<"FRAME_ID : "<<transformStamped.header.frame_id<<endl;
		cout<<"CHILD_FRAME_ID : "<<transformStamped.child_frame_id<<endl;
		cout<<"[TRANSLATION]"<<endl<<"x : "<<transformStamped.transform.translation.x<<endl;
		cout<<"y : "<<transformStamped.transform.translation.y<<endl;
		cout<<"z : "<<transformStamped.transform.translation.z<<endl;
		cout<<"[ROTATION]"<<endl<<"x :"<<transformStamped.transform.rotation.x<<endl;
		cout<<"y : "<<transformStamped.transform.rotation.y<<endl;
		cout<<"z : "<<transformStamped.transform.rotation.z<<endl;
		cout<<"w : "<<transformStamped.transform.rotation.w<<endl;
		cout<<"SEND!!"<<endl;
	}
	return true;
}
bool operate_2(assembly_robot_msgs::cam_Srv::Request &req,assembly_robot_msgs::cam_Srv::Response &res){
	using namespace std;
	string number[10] ={"1","2","3","4","5","6","7","8","9","10"}; 
	cout<<"RECIVED_DATA = "<<req.name<<endl;
	geometry_msgs::TransformStamped transform,hole_tf;
	queue<geometry_msgs::TransformStamped> tf_q;
	string hole_name = req.name;
	string part_num = hole_name.substr(0,6);
	vector<pair<int,int>> left_pair,right_pair;
	vector<pair<double,double>> tf_list;
	try{
		for(int i=0;i != 10;i++){
			hole_tf = tfBuffer.lookupTransform("camera_center_2",hole_name,ros::Time(0));
			transform = tfBuffer.lookupTransform("camera_center_2",part_num+number[i],ros::Time(0));
			cout<<part_num+number[i]<<endl;
			tf_q.push(transform);
		}
	}
	catch (tf::TransformException ex){
		if(tf_q.empty()){
			ROS_ERROR("%s",ex.what());
			return false;
		}
		else
			ROS_INFO("FOUND %d HOLSE",(int)tf_q.size());
	}
	int q_size = tf_q.size();
	for(int i=0;i<q_size;i++){
		geometry_msgs::TransformStamped temp = tf_q.front();
		tf_q.pop();
		cout<<abs(temp.transform.translation.x - hole_tf.transform.translation.x)<<"\t"<<abs(temp.transform.translation.y - hole_tf.transform.translation.y)<<"\t"<<abs(temp.transform.translation.z - hole_tf.transform.translation.z)<<endl;
		if(abs(temp.transform.translation.x - hole_tf.transform.translation.x)<0.1 && abs(temp.transform.translation.y - hole_tf.transform.translation.y)<0.1 && abs(temp.transform.translation.z - hole_tf.transform.translation.z)<0.03){
			pair<pair<int,int>,pair<int,int>> pair_temp;
			cout<<temp.transform.translation.x<<" wowow "<<temp.transform.translation.y<<endl;
			tf::Quaternion quat;
    		tf::quaternionMsgToTF(temp.transform.rotation, quat);
    		double roll, pitch, yaw;
    		tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
			cout<<roll<<" "<<pitch<<" "<<yaw<<endl;
			pair_temp = reverse_to_pixel(-temp.transform.translation.x,-temp.transform.translation.y,temp.transform.translation.z,cam1);
			left_pair.push_back(pair_temp.second);
			right_pair.push_back(pair_temp.first);
			cout<<endl<<pair_temp.second.first<<" "<<pair_temp.second.second<<" "<<pair_temp.first.first<<" "<<pair_temp.first.second<<endl<<endl;
			tf_list.push_back(make_pair(temp.transform.translation.x,temp.transform.translation.y));
		}
	}
	sort(tf_list.begin(),tf_list.end());
	sort(left_pair.begin(),left_pair.end());
	sort(right_pair.begin(),right_pair.end());
	int to_find_index=tf_list.size() -1;
	for(int i=0;i<tf_list.size();i++){
		if(tf_list[i].first == hole_tf.transform.translation.x && tf_list[i].second==hole_tf.transform.translation.y)
			break;
		to_find_index--;
	}
	double output[3] {0,0,0};
	cout<<"======================================================"<<endl;
	cout<<"====================== SERVICE START=================="<<endl<<endl;
	cout<<"HOLE NUM : "<<tf_list.size()<<endl;
	cout<<"FIND INDEX : "<<to_find_index<<endl;
	cout<<"============ FIND ==============="<<endl;
	res.result =0;
	res.result = finder_2->find_circle_tf(output,tf_list.size(),to_find_index,left_pair,right_pair);
	cout<<"============ RESULT ============="<<endl<<endl;
	cout<<"INSERT? : "<<(int)res.result<<endl;
	res.name = "camera_center_2";
	res.x = -output[0]/1000;
	res.y = -output[1]/1000;
	res.z = -output[2]/1000;
	cout<<"POSITNON \t : [ "<<res.x<<" , "<<res.y<<" , "<<res.z<<"]"<<endl<<endl;
	if(!res.result && !output[0]&& !output[1]&& !output[2])
		return false;
	return true;
}
#endif