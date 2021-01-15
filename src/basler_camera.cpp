#include "ros/ros.h"
#include "basler.cpp"
#include "image_transport/image_transport.h"
#include "sensor_msgs/CameraInfo.h"
int main(int argc, char** argv){
	Basler cam;
	cam.get_camera();
	ros::init(argc,argv,"Basler_Camera");
	ros::NodeHandle nh;
	ros::Publisher L_info_pub = nh.advertise<sensor_msgs::CameraInfo>("basler/info/left",1);
	ros::Publisher R_info_pub = nh.advertise<sensor_msgs::CameraInfo>("basler/info/right",1);
	image_transport::ImageTransport it(nh);
	image_transport::Publisher L_pub = it.advertise("basler/image_left", 1);
	image_transport::Publisher R_pub = it.advertise("basler/image_right", 1);
	sensor_msgs::CameraInfo cam_info_1,cam_info_2;
	cam_info_1.header.frame_id = "left_camera";
	cam_info_2.header.frame_id = "right_camera";
	cam.start();
	while (nh.ok()) {
		cam.start();
		cam_info_1.header.stamp = ros::Time::now();
		cam_info_2.header.stamp = ros::Time::now();
		cam_info_1.height = cam.opencvImage1.rows;
		cam_info_2.height = cam.opencvImage2.rows;
		cam_info_1.width = cam.opencvImage1.cols;
		cam_info_2.height = cam.opencvImage2.cols;
		sensor_msgs::ImagePtr msg_1 = cv_bridge::CvImage(std_msgs::Header(), "rgb8", cam.opencvImage1).toImageMsg();
		sensor_msgs::ImagePtr msg_2 = cv_bridge::CvImage(std_msgs::Header(), "rgb8", cam.opencvImage2).toImageMsg();
		L_info_pub.publish(cam_info_1);
		R_info_pub.publish(cam_info_2);
		L_pub.publish(msg_1);
		R_pub.publish(msg_2);
    }
	return 0;
}