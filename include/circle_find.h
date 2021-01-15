#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
#include "std_msgs/Float32.h"
#include "geometry_msgs/TransformStamped.h"
#include "assembly_robot_msgs/Hole_poses.h"
#include "img_tf.h"
#include <set>
#include <map>
#include <cmath>
#include <fstream>
using namespace cv;
using namespace std;
typedef vector<pair<int,int>> Pattern;
typedef multimap<double,Pattern> Score;
typedef multimap<double,pair<Pattern,Pattern>> Paring;
typedef map<pair<double,double>,geometry_msgs::Vector3,greater<pair<double,double>>> HoleList;
class CircleFind
{
public:
    CircleFind(string topicname[2],ImgTfFinder::CameraInfo cam_info[2]);
    ~CircleFind();
    void circle_t(Mat &input,vector<Vec3f> &circles,Size blur_size,int canny_1,int canny_2,int Threshold,string image_name);
    vector<Vec3f> circles[2];
    Mat opencvImage1,opencvImage2;
    bool find_circle_tf(double dst[3],HoleList &src,int hole_num,int find_num,int missing);
private:
    int img_index;
    ofstream outFile;
	ros::ServiceClient yoloy_client;
	sensor_msgs::Image Image_1,Image_2;
    assembly_robot_msgs::Hole_poses hole_pixel;
    ImgTfFinder *tfFinder;
    ros::NodeHandle nh_;
    cv_bridge::CvImagePtr cv_ptr;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_1,image_sub_2;
    string File_Path;
    string topic_name[2];
    ImgTfFinder::CameraInfo cam[2];
    Score line_matching(vector<Pattern>&src,int pt_size);
    Paring diff_matching(Paring &src,int pt_size);
    Paring Y_matching(Paring &src,int pt_size);
    Paring image_matching(Score &dst,Score &src,int pt_size);
    vector<Pattern> yolo_case(set<pair<int,int>> &src,int pt_size);
    void show_scoreImg(Mat &img,Pattern &src,string name,int hole_num,int find);
    void show_circles(Mat &img,vector<Vec3f> &circle,string name);
    vector<Pattern> find_case(vector<Vec3f> &circle, int pt_size);
    vector<Vec3f> sorting(vector<Vec3f> &circles);
    void imageCb_1(const sensor_msgs::ImageConstPtr& msg);
    void imageCb_2(const sensor_msgs::ImageConstPtr& msg);
};







