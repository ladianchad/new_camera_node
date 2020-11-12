#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
#include "std_msgs/Float32.h"
#include "geometry_msgs/TransformStamped.h"
#include "img_tf.h"
#include <set>
#include <map>
#include <cmath>
using namespace cv;
using namespace std;
typedef vector<pair<int,int>> Pattern;
typedef multimap<double,Pattern> Score;
class CircleFind
{
public:

    CircleFind(string topicname[2],ImgTfFinder::CameraInfo cam_info[2]);
    ~CircleFind();
    void get_img();
    static bool cmp(const Vec3f& p1, const Vec3f& p2);
    static bool cmp_1(const Vec3f& p1, const Vec3f& p2);
    void circle_t(Mat &input,vector<Vec3f> &circles,Size blur_size,int canny_1,int canny_2,int Threshold,string image_name);
    vector<Vec3f> circles[2];
    Mat opencvImage1,opencvImage2;
    bool find_circle_tf(double dst[3],int hole_num,int finde_num,vector<pair<int,int>> &L_boundary,vector<pair<int,int>> &R_boundary);
private:
    int distance;
    int img_index;
    string file_path;
    ImgTfFinder *tfFinder;
    ros::NodeHandle nh_;
    cv_bridge::CvImagePtr cv_ptr;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_1,image_sub_2;
    int minDist;
    int minRadius;
    int maxRadius; 
    string topic_name[2];
    ros::Subscriber test_sub1,test_sub2;
    void data_save();
    Score pattern_accuracy_check(vector<Pattern> &list,Pattern &pattern);
    Score pattern_accuracy_check(vector<Pattern> &dst,vector<Pattern> &src, Pattern &pattern);
    Pattern make_pattern(Pattern &input);
    Pattern make_pattern(Pattern &input1,Pattern &input2);
    vector<Pattern> find_case(vector<Vec3f> &circles, Pattern &pattern);
    void sorting(vector<Vec3f> &circles);
    void imageCb_1(const sensor_msgs::ImageConstPtr& msg);
    void imageCb_2(const sensor_msgs::ImageConstPtr& msg);
};







