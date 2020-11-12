#ifndef BASLER_H
#define BASLER_H
#include "basler.h"

Basler::Basler() : cameras(NULL)
{
    converter.OutputPixelFormat = PixelType_BGR8packed;
    Camera_matrix_L=(Mat1d(3, 3) << 697.159010, 0., 316.611186, 0., 697.505723, 258.052694, 0., 0., 1.);
    dist_L = (Mat1d(1,5) << -0.226470, 0.096907, 0.003841, -0.002965, 0.000000);
    Camera_matrix_R=(Mat1d(3, 3) << 669.484618, 0.000000, 322.411075, 0.000000, 670.458375, 248.179269, 0.000000, 0.000000, 1.000000);
    dist_R = (Mat1d(1,5) << -0.224482, 0.180080, 0.004914, -0.008142, 0.000000);
}
Basler::~Basler()
{
    PylonTerminate();
    delete cameras;
}
void Basler::get_camera()
{
    PylonInitialize();
    CTlFactory& tlFactory = CTlFactory::GetInstance();
    if ( tlFactory.EnumerateDevices(devices) == 0 )
            cout<<"No camera present!!"<<endl;
    cameras = new CInstantCameraArray( min( devices.size(), Basler::c_maxCamerasToUse));
        for ( size_t i = 0; i < cameras->GetSize(); ++i)
            {
            (*cameras)[i].MaxNumBuffer = 10;
                (*cameras)[ i ].Attach( tlFactory.CreateDevice( devices[ i ]));
                cout << "Using device " << (*cameras)[ i ].GetDeviceInfo().GetModelName() << endl;
            }
    cameras->StartGrabbing();
}

void Basler::start()
{
    bool ch1 = false;
    bool ch2 = false;
    bool cam_flag_1 = false;
    bool cam_flag_2 = false;
    while(!ch1 || !ch2)
    {
        if (cameras->IsGrabbing())
            {
            ///// camera Data get
	    
            cameras->RetrieveResult( 5000, ptrGrabResult, TimeoutHandling_ThrowException);
		    int Succeded = ptrGrabResult->GrabSucceeded();
            cout<<Succeded<<endl;
            if(Succeded)
            {
		
                static intptr_t cameraContextValue;
                cameraContextValue = ptrGrabResult->GetCameraContext();
                cout<<cameraContextValue<<endl;
                if(cameraContextValue==0 &&ch1 == false)
                {
                    converter.Convert(pylonImage,ptrGrabResult);
                    temp_img1 = cv::Mat(ptrGrabResult->GetHeight(),ptrGrabResult->GetWidth(),CV_8UC3,(uint8_t *)pylonImage.GetBuffer());
                    cam_flag_1 = true;
                }
                if(cameraContextValue==1&&ch2 == false)
                {
                    converter.Convert(pylonImage,ptrGrabResult);
                    temp_img2 = cv::Mat(ptrGrabResult->GetHeight(),ptrGrabResult->GetWidth(),CV_8UC3,(uint8_t *)pylonImage.GetBuffer());
                    cam_flag_2 = true;
                }
                if (temp_img1.empty() && temp_img2.empty())
                {
                    cerr << "Image load failed!" << endl;
                    return ;
                }
                else  // opencv funtions
                {

                    if(!temp_img1.empty() && !ch1 && !cam_flag_1){
                        int origin_s_c = temp_img1.cols;
                        int origin_s_r = temp_img1.rows;
                        resize(temp_img1, temp_img1, Size(temp_img1.cols/4, temp_img1.rows/4), 0, 0, CV_INTER_NN);
                        if(temp_img1.cols == origin_s_c/4 && temp_img1.rows == origin_s_r/4){
                            undistort(temp_img1, opencvImage1, Camera_matrix_L, dist_L);
                            imshow("image_1",opencvImage1);
                            waitKey(1);
                            ch1 = true;
                            cam_flag_1 = true;
                        }
                        else{
                            cam_flag_1 = false;
                        }
                    }
                    if(!temp_img2.empty()&& !ch2 && !cam_flag_2){
                        int origin_s_c = temp_img2.cols;
                        int origin_s_r = temp_img2.rows;
                        resize(temp_img2, temp_img2, Size(temp_img2.cols/4, temp_img2.rows/4), 0, 0, CV_INTER_NN);
                        if(temp_img2.cols == origin_s_c/4 && temp_img2.rows == origin_s_r/4){
                            undistort(temp_img2, opencvImage2, Camera_matrix_R, dist_R);
                            imshow("image_2",opencvImage2);
                            waitKey(1);
                            ch2 = true;
                            cam_flag_2 = true;
                        }
                        else{
                            cam_flag_2 = false;
                        }
                    }
                }
            } // end of else opencvImage.empty()
        }
    }
}
#endif // BASLER_H
