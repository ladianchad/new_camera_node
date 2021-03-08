#include "ros/ros.h"
#include "circle_find.cpp"
#include <tf2_ros/transform_listener.h>
#include <tf/transform_listener.h>
#include "assembly_robot_msgs/cam_Srv.h"
#include <string>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

using namespace std;
using namespace cv;
typedef assembly_robot_msgs::cam_Srv::Request Srv_in;
typedef assembly_robot_msgs::cam_Srv::Response Srv_out;
typedef geometry_msgs::TransformStamped TF;
bool operate_1(Srv_in &req,Srv_out &res);
bool operate_2(Srv_in &req,Srv_out &res);
Mat eulerAnglesToRotationMatrix(Vec3f &theta);
void service(int mode);
ImgTfFinder::CameraInfo cam1[2]{ {979.695288, 549.913691, 1448.769366,1445.308339}
    , {963.129163,583.055193,1442.776223,1441.511784}};
ImgTfFinder::CameraInfo cam2[2]{ {966.616058,617.093752,1459.348839,1457.665754}
    , {1010.841717, 544.825729, 1461.046378,1461.657982}};
tf2_ros::Buffer tfBuffer;
tf2_ros::TransformListener *tfListener;
CircleFind *finder_1;
CircleFind *finder_2;
int main(int argc, char** argv){
	ros::init(argc, argv, "hole_finder");
	ros::NodeHandle nh;
	string topic_1[2] = { "/camera1/usb_cam1_2/image_raw","/camera1/usb_cam1_1/image_raw"};
	string topic_2[2] = { "/camera2/usb_cam2_2/image_raw","/camera2/usb_cam2_1/image_raw"};
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

bool operate_1(Srv_in &req,Srv_out &res){
	cout<<"I'M SERVER 1"<<endl;
	cout<<"RECIVED_DATA = "<<req.name<<endl;
	TF transform,hole_tf;
	vector<TF> holes_tf;
	map<pair<double,double>,geometry_msgs::Vector3,greater<pair<double,double>>> to_find_holse;
	string hole_name = req.name;
	string part_num = hole_name.substr(0,13);
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
	Vec3f hole_vector;
	tf::quaternionMsgToTF(hole_tf.transform.rotation,quat_temp);
	tf::Matrix3x3(quat_temp).getRPY(hole_r,hole_p,hole_y);
	cout<<"[ MY HOLE ]"<<endl;
	cout<<"transform"<<endl;
	cout<<hole_tf.transform.translation.x<<" "<<hole_tf.transform.translation.y<<" "<<hole_tf.transform.translation.z<<endl;
	cout<<"rotation"<<endl;
	cout<<hole_r<<" "<<hole_p<<" "<<hole_y<<endl;

	int missing_hole = 0;
	Vec3f temp_angle;
	temp_angle[0] = hole_r;
	temp_angle[1] = hole_p;
	temp_angle[2] = hole_y;
	Vec3f z_axis = (0,0,1);
	Mat rotation_matrix = eulerAnglesToRotationMatrix(temp_angle);
	rotation_matrix*Mat(z_axis);
	hole_vector[0] = rotation_matrix.at<double>(0,0);
	hole_vector[1] = rotation_matrix.at<double>(1,1);
	hole_vector[2] = rotation_matrix.at<double>(2,2);
	cout<<endl<<"hole_vector"<<endl;
	cout<<hole_vector<<endl;
	for(int i=0;i<holes_tf.size();i++){
		geometry_msgs::Vector3 pose_temp;
		pose_temp = holes_tf[i].transform.translation;
		if(sqrt(pow(hole_pose.x - pose_temp.x,2) + pow(hole_pose.y - pose_temp.y,2))<0.15 && abs(hole_pose.z - pose_temp.z)<0.07){
			tf::quaternionMsgToTF(holes_tf[i].transform.rotation, quat_temp);
			double r,p,y;
			tf::Matrix3x3(quat_temp).getRPY(r,p,y);
			cout<<"HOLE"<<i+1<<endl;
			cout<<"r : "<<r<<" p : "<<p<<" y : "<<y<<endl;
			temp_angle[0] = r;
			temp_angle[1] = p;
			temp_angle[2] = y;
			rotation_matrix = eulerAnglesToRotationMatrix(temp_angle);
			z_axis[0] = 0;
			z_axis[1] = 0;
			z_axis[2] = 1;
			rotation_matrix*Mat(z_axis);
			Vec3f temp_vector; 
			temp_vector[0] = rotation_matrix.at<double>(0,0);
			temp_vector[1] = rotation_matrix.at<double>(1,1);
			temp_vector[2] = rotation_matrix.at<double>(2,2);
			cout<<"vector"<<endl;
			cout<<temp_vector<<endl;
			double cross_result = temp_vector[0]*hole_vector[0]+temp_vector[1]*hole_vector[1]+temp_vector[2]*hole_vector[2];
			cout<<"cross result"<<endl;
			cout<<cross_result<<endl;
			if(cross_result<0)
				missing_hole++;
			if(abs(cross_result)>0.8)
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
	double output[3] {};
	res.result = finder_1->find_circle_tf(output,to_find_holse,Hole_num,find_index,missing_hole);
	res.name = "camera_center_1";
	res.x = -output[0]/1000;
	res.y = -output[1]/1000;
	res.z = output[2]/1000;
	cout<<"END SERVICE!!"<<endl;
	if(res.result){
		cout<<"GOOD!!!\t\tSEND!!!"<<endl;
		return true;
	}
	else if(output[0] == 0 && output[1] == 0 &&output[2] == 0){
		cout<<"NO MATCHING !!"<<endl;
		return false;
	}
	else{
		cout<<"MOVE!!"<<endl;
		return true;
	}
}
bool operate_2(Srv_in &req,Srv_out &res){
	cout<<"I'M SERVER 2"<<endl;
	cout<<"RECIVED_DATA = "<<req.name<<endl;
	TF transform,hole_tf;
	vector<TF> holes_tf;
	map<pair<double,double>,geometry_msgs::Vector3,greater<pair<double,double>>> to_find_holse;
	string hole_name = req.name;
	string part_num = hole_name.substr(0,13);
	try{ 
		hole_tf = tfBuffer.lookupTransform("camera_center_2",hole_name,ros::Time(0));
		for(int i=1;i < 11;i++){
			transform = tfBuffer.lookupTransform("camera_center_2",part_num+to_string(i),ros::Time(0));
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
	Vec3f hole_vector;
	tf::quaternionMsgToTF(hole_tf.transform.rotation,quat_temp);
	tf::Matrix3x3(quat_temp).getRPY(hole_r,hole_p,hole_y);
	cout<<"[ MY HOLE ]"<<endl;
	cout<<"transform"<<endl;
	cout<<hole_tf.transform.translation.x<<" "<<hole_tf.transform.translation.y<<" "<<hole_tf.transform.translation.z<<endl;
	cout<<"rotation"<<endl;
	cout<<hole_r<<" "<<hole_p<<" "<<hole_y<<endl;

	int missing_hole = 0;
	Vec3f temp_angle;
	temp_angle[0] = hole_r;
	temp_angle[1] = hole_p;
	temp_angle[2] = hole_y;
	Vec3f z_axis = (0,0,1);
	Mat rotation_matrix = eulerAnglesToRotationMatrix(temp_angle);
	rotation_matrix*Mat(z_axis);
	hole_vector[0] = rotation_matrix.at<double>(0,0);
	hole_vector[1] = rotation_matrix.at<double>(1,1);
	hole_vector[2] = rotation_matrix.at<double>(2,2);
	cout<<"hole_vector"<<endl;
	cout<<hole_vector<<endl;
	for(int i=0;i<holes_tf.size();i++){
		geometry_msgs::Vector3 pose_temp;
		pose_temp = holes_tf[i].transform.translation;
		if(sqrt(pow(hole_pose.x - pose_temp.x,2) + pow(hole_pose.y - pose_temp.y,2))<0.15 && abs(hole_pose.z - pose_temp.z)<0.07){
			tf::quaternionMsgToTF(holes_tf[i].transform.rotation, quat_temp);
			double r,p,y;
			tf::Matrix3x3(quat_temp).getRPY(r,p,y);
			cout<<endl<<"HOLE"<<i+1<<endl;
			cout<<"r : "<<r<<" p : "<<p<<" y : "<<y<<endl;
			temp_angle[0] = r;
			temp_angle[1] = p;
			temp_angle[2] = y;
			rotation_matrix = eulerAnglesToRotationMatrix(temp_angle);
			z_axis[0] = 0;
			z_axis[1] = 0;
			z_axis[2] = 1;
			rotation_matrix*Mat(z_axis);
			Vec3f temp_vector; 
			temp_vector[0] = rotation_matrix.at<double>(0,0);
			temp_vector[1] = rotation_matrix.at<double>(1,1);
			temp_vector[2] = rotation_matrix.at<double>(2,2);
			cout<<"vector"<<endl;
			cout<<temp_vector<<endl;
			double cross_result = temp_vector[0]*hole_vector[0]+temp_vector[1]*hole_vector[1]+temp_vector[2]*hole_vector[2];
			cout<<"cross result"<<endl;
			cout<<cross_result<<endl;
			if(cross_result<0)
				missing_hole++;
			if(abs(cross_result)>0.8)
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
	double output[3] {} ;
	res.result = finder_2->find_circle_tf(output,to_find_holse,Hole_num,find_index,missing_hole);
	res.name = "camera_center_2";
	res.x = output[0]/1000;
	res.y = output[1]/1000;
	res.z = output[2]/1000;
	cout<<"END SERVICE!!"<<endl;
	if(res.result){
		cout<<"GOOD!!!\t\tSEND!!!"<<endl;
		return true;
	}
	else if(output[0] == 0 && output[1] == 0 &&output[2] == 0){
		cout<<"NO MATCHING !!"<<endl;
		return false;
	}
	else{
		cout<<"MOVE!!"<<endl;
		return true;
	}
}
Mat eulerAnglesToRotationMatrix(Vec3f &theta){
    Mat R_x = (Mat_<double>(3,3) <<
               1,       0,              0,
               0,       cos(theta[0]),   -sin(theta[0]),
               0,       sin(theta[0]),   cos(theta[0])
               );
 
    Mat R_y = (Mat_<double>(3,3) <<
               cos(theta[1]),    0,      sin(theta[1]),
               0,               1,      0,
               -sin(theta[1]),   0,      cos(theta[1])
               );
    Mat R_z = (Mat_<double>(3,3) <<
               cos(theta[2]),    -sin(theta[2]),      0,
               sin(theta[2]),    cos(theta[2]),       0,
               0,               0,                  1);
    Mat R = R_z * R_y * R_x;
    return R;
}