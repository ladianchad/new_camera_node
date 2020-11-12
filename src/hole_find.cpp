#include "ros/ros.h"
#include "circle_find.cpp"
#include <tf2_ros/transform_listener.h>
#include <tf/transform_listener.h>
#include "assembly_robot_msgs/cam_Srv.h"
#include <string>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

using namespace std;
typedef assembly_robot_msgs::cam_Srv::Request Srv_in;
typedef assembly_robot_msgs::cam_Srv::Response Srv_out;
typedef geometry_msgs::TransformStamped TF;
bool operate_1(Srv_in &req,Srv_out &res);
void service(int mode);
ImgTfFinder::CameraInfo cam1[2]{ {931.3824090000001,560.6960309999999,1460.136731,1459.304695}
    , {937.655922, 558.151482, 1475.831374,1473.475273}};
ImgTfFinder::CameraInfo cam2[2]{ {1014.006763,526.753957,1467.924445,1465.315384}
    , {959.665281, 593.683847, 1469.472732,1467.230973}};
tf2_ros::Buffer tfBuffer;
tf2_ros::TransformListener *tfListener;
CircleFind *finder_1;
CircleFind *finder_2;
int main(int argc, char** argv){
	ros::init(argc, argv, "hole_finder");
	ros::NodeHandle nh;
	string topic_1[2] = { "/camera1/usb_cam1_1/image_raw","/camera1/usb_cam1_2/image_raw"};
	string topic_2[2] = { "/camera2/usb_cam2_1/image_raw","/camera2/usb_cam2_2/image_raw"};
	finder_1 = new CircleFind(topic_1,cam1);
	finder_2 = new CircleFind(topic_2,cam2);
	tfListener = new tf2_ros::TransformListener(tfBuffer);
	ros::ServiceServer cam_server_1 = nh.advertiseService("camera_server_1",operate_1);
	//ros::ServiceServer cam_server_2 = nh.advertiseService("camera_server_2",operate_2);
	ROS_INFO("READY CAMERA SERVER");
	ros::spin();	
	delete finder_1,finder_2,tfListener;
	return 0;
}

bool operate_1(Srv_in &req,Srv_out &res){
	cout<<"RECIVED_DATA = "<<req.name<<endl;
	TF transform,hole_tf;
	vector<TF> holes_tf;
	map<pair<double,double>,geometry_msgs::Vector3> to_find_holse;
	string hole_name = req.name;
	string part_num = hole_name.substr(0,6);
	try{ 
		hole_tf = tfBuffer.lookupTransform("camera_center_1",hole_name,ros::Time(0));
		for(int i=1;i < 11;i++){
			transform = tfBuffer.lookupTransform("camera_center_1",part_num+to_string(i),ros::Time(0));
			holes_tf.push_back(transform);
		}
	}
	catch (tf::TransformException ex){
		if(holes_tf.empty()){
			ROS_ERROR("%s",ex.what());
			return false;
		}
		else
			ROS_INFO("FOUND %d HOLSE",(int)holes_tf.size());
	}
	tf::Quaternion quat_temp;
	geometry_msgs::Vector3 hole_pose;
	hole_pose = hole_tf.transform.translation;
	double hole_r,hole_p,hole_y;
	tf::quaternionMsgToTF(hole_tf.transform.rotation,quat_temp);
	tf::Matrix3x3(quat_temp).getRPY(hole_r,hole_p,hole_y);
	cout<<"[ MY HOLE ]"<<endl;
	cout<<"transform"<<endl;
	cout<<hole_tf.transform.translation.x<<" "<<hole_tf.transform.translation.y<<" "<<hole_tf.transform.translation.z<<endl;
	cout<<"rotation"<<endl;
	cout<<hole_r<<" "<<hole_p<<" "<<hole_y<<endl;
	for(int i=0;i<holes_tf.size();i++){
		geometry_msgs::Vector3 pose_temp;
		pose_temp = holes_tf[i].transform.translation;
		if(sqrt(pow(hole_pose.x - pose_temp.x,2) + pow(hole_pose.y - pose_temp.y,2))<0.1 && abs(hole_pose.z - pose_temp.z)<0.03){
			tf::quaternionMsgToTF(holes_tf[i].transform.rotation, quat_temp);
			double r,p,y;
			tf::Matrix3x3(quat_temp).getRPY(r,p,y);
			if(abs(abs(y) - abs(hole_y))<0.01)
				to_find_holse.insert(make_pair(make_pair(pose_temp.x,pose_temp.y),pose_temp));
		}
	}
	int Hole_num = to_find_holse.size();
	int find_index = 0;
	for(auto iter = to_find_holse.begin();iter !=to_find_holse.end();iter++){
		if(iter->second == hole_pose)
			break;
		find_index++;
	}
	cout<<"TO FIND HOLES NUM : "<<Hole_num<<endl;
	cout<<"FIND INDEX: "<<find_index<<endl;
	cout<<"====================== FIND HOLE START ======================"<<endl<<endl;
	double output[3];
	res.result = finder_1->find_circle_tf(output,to_find_holse,Hole_num,find_index);
}
