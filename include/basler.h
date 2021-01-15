
#include <pylon/PylonIncludes.h>
#ifdef PYLON_WIN_BUILD
#    include <pylon/PylonGUI.h>
#endif
#include <opencv2/opencv.hpp>
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"


using namespace cv;
using namespace std;
using namespace Pylon;

class Basler
{

public:
    Basler();
    ~Basler();
    void get_camera();
    void start();
    Mat opencvImage1,opencvImage2;
private:
    size_t c_maxCamerasToUse = 2;
    Pylon::PylonAutoInitTerm autoInitTerm;
    CImageFormatConverter converter;
    CPylonImage pylonImage;
    CInstantCameraArray* cameras = NULL;
    DeviceInfoList_t devices;
    CGrabResultPtr ptrGrabResult;
    Mat temp_img1,temp_img2;
    Mat Camera_matrix_L = Mat::eye(3,3,CV_64FC1); 
    Mat dist_L = Mat::zeros(1,5,CV_64FC1); 
    Mat Camera_matrix_R = Mat::eye(3,3,CV_64FC1); 
    Mat dist_R = Mat::zeros(1,5,CV_64FC1);
    intptr_t cameraContextValue;
};






