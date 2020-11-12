#ifndef CIRCLE_FIND_H
#define CIRCLE_FIND_H
#include "circle_find.h"
#include <fstream>
#include <stdlib.h>
#include <string>
CircleFind::CircleFind(string topicname[2],ImgTfFinder::CameraInfo cam_info[2]): it_(nh_)
{
  topic_name[0] = topicname[0];
  topic_name[1] = topicname[1];
  image_sub_1 = it_.subscribe(topic_name[0], 1 , &CircleFind::imageCb_1, this);
  image_sub_2= it_.subscribe(topic_name[1], 1 , &CircleFind::imageCb_2, this);
  tfFinder = new ImgTfFinder(cam_info[0],cam_info[1],30,0);
}

CircleFind::~CircleFind(){
	delete tfFinder;
}

void CircleFind::imageCb_1(const sensor_msgs::ImageConstPtr& msg)
{
     try
     {
       cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
     }
     catch (cv_bridge::Exception& e)
      {
       ROS_ERROR("cv_bridge exception: %s", e.what());
       return;
     }
  
	cv_ptr->image.copyTo(opencvImage1);
	Mat to_show;
	resize(opencvImage1,to_show, cv::Size( 720, 405), 0, 0, CV_INTER_NN );
    imshow(topic_name[0],to_show);
    waitKey(1);
}

void CircleFind::imageCb_2(const sensor_msgs::ImageConstPtr& msg)
{
     try
     {
       cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
     }
     catch (cv_bridge::Exception& e)
      {
       ROS_ERROR("cv_bridge exception: %s", e.what());
       return;
     }
     cv_ptr->image.copyTo(opencvImage2);
     Mat to_show;
	 resize(opencvImage2,to_show, cv::Size( 720, 405), 0, 0, CV_INTER_NN );
     imshow(topic_name[1],to_show );
     waitKey(1);
}

void CircleFind::circle_t(Mat &input,vector<Vec3f> &circles,Size blur_size,int canny_1,int canny_2,int Threshold,string image_name) {
    Mat edge;
    circles.clear();
    Mat gray;
    cvtColor(input, gray, COLOR_BGR2GRAY);
 	GaussianBlur(input, input, Size(7,7), 0);
    Canny(input, edge, canny_1,canny_2,3,true);
    dilate(edge, edge, cv::Mat());
    GaussianBlur(edge, edge, blur_size, 0);
    dilate(edge, edge, cv::Mat());
    GaussianBlur(edge, edge, blur_size, 0);
    dilate(edge, edge, cv::Mat());
    HoughCircles(edge, circles, HOUGH_GRADIENT, 1,100,10, 30, 3, 40);
    resize(edge,edge, cv::Size( 720, 405), 0, 0, CV_INTER_NN );
    imshow( image_name + "edge",edge);
    sorting(circles);
}

bool CircleFind::find_circle_tf(double dst[3],HoleList &src,int hole_num,int find_num){
	circle_t(opencvImage1,circles[0],Size(9,9),0,45,50,topic_name[0] +"/circle_1");
	circle_t(opencvImage2,circles[1],Size(9,9),0,45,150,topic_name[1] +"/circle_2");
	cout<<"LEFT CIRECLE FIND : "<<circles[0].size()<<endl;
	cout<<"RIGHT CIRECLE FIND : "<<circles[1].size()<<endl;
	if(circles[0].size() < hole_num || circles[1].size() < hole_num){
		cout<<endl<<"!!!ERORR!!!"<<endl<<"CAN'T FIND HOLES!!"<<endl;
		return false;
	}
	show_circles(opencvImage1,circles[0],topic_name[0] + " circles");
	show_circles(opencvImage2,circles[1],topic_name[1] + " circles");
	vector<Pattern> left_case,right_case;
	left_case = find_case(circles[0],hole_num);
	right_case = find_case(circles[1],hole_num);

	Score L_score = line_matching(left_case,hole_num);
	Score R_score = line_matching(right_case,hole_num);

	Paring score = image_matching(L_score,R_score,src,hole_num);
	cout<<"Matching Complete"<<endl;
	cout<<score.size()<<endl;
	show_scoreImg(opencvImage1,score.begin()->second.first,topic_name[0] ,hole_num);
	show_scoreImg(opencvImage2,score.begin()->second.second,topic_name[1],hole_num);
}
void CircleFind::show_scoreImg(Mat &img,Pattern &src,string name,int hole_num){
	Mat temp;
	img.copyTo(temp);
	int count =0;
	for(int i=0;i<hole_num;i++)
		cv::circle(temp, Point(src[i].first,src[i].second),15,Scalar(255,0,0),3);
	resize(temp,temp, cv::Size( 720, 405), 0, 0, CV_INTER_NN );
	imshow(name+"high score",temp);
}
Paring CircleFind::image_matching(Score &dst,Score &src,HoleList hole_list,int pt_size){
	Paring output;
	for(auto dst_it = dst.begin();dst_it != dst.end();dst_it++){
		for(auto src_it = src.begin();src_it != src.end(); src_it++){
			double err =0;
			for (int i = 1; i < pt_size; ++i){
				double dst_dx = dst_it->second[i].first - dst_it->second[i-1].first;
				double dst_dy = dst_it->second[i].second - dst_it->second[i-1].second;
				double src_dx = src_it->second[i].first - src_it->second[i-1].first;
				double src_dy = src_it->second[i].second - src_it->second[i-1].second;
				err += (abs(dst_dx - src_dx) + abs(src_dy - dst_dy));
			}
			output.insert(make_pair(err,make_pair(dst_it->second,src_it->second)));
		}
	}
	return output;
}
Score CircleFind::line_matching(vector<Pattern>&src,int pt_size){
	Score output;
	for(int i=0;i< src.size();i++){
		double pre_dx = src[i][1].first - src[i][0].first;;
		double pre_dy = src[i][1].second - src[i][0].second;
		double err =0;
		for(int k=2;k<pt_size;k++){
			double dx = src[i][k].first - src[i][k-1].first;
			double dy = src[i][k].second - src[i][k-1].second;
			err += (abs(dx - pre_dx)+abs(dy - pre_dy));
		}
		if(err < 50)
			output.insert(make_pair(err,src[i]));
	}
	return output;
} 
void CircleFind::show_circles(Mat &img,vector<Vec3f> &circle,string name){
	Mat temp;
	img.copyTo(temp);
	for(int i=0; i<circle.size();i++){
    	cv::circle(temp, Point(circle[i][0],circle[i][1]),circle[i][2],Scalar(255,0,0),3);
  	}
  	resize(temp,temp, cv::Size( 720, 405), 0, 0, CV_INTER_NN );
  	imshow(name,temp);
  	waitKey(1);
}
vector<Pattern> CircleFind::find_case(vector<Vec3f> &circle, int pt_size){
	vector<int> cases;
	vector<Pattern> output;
	int n = circle.size();
	for(int i=0;i<pt_size;i++){
		cases.push_back(i);
	}
	while(true){
	  	Pattern temp;
	  	for(int i=0;i<pt_size;i++){
	  		temp.push_back(make_pair(circle[cases[i]][0],circle[cases[i]][1]));
	  	}
	  	output.push_back(temp);
	  	int end =0;
	  	for(int i=0;i<cases.size();i++){
	  		if(cases[i] == n - pt_size +i)
	  			end ++;
	  	}
	  	if(end == cases.size())
	  		break;
	  	cases[pt_size-1]++;
	  	if(cases[pt_size-1] > n-1){
	  		for(int i=pt_size-2;i>0;i--){
	  			cases[i]++;
	  			if(cases[i] < n-1){
	  				for(int j=i+1;j<pt_size;j++){
	  					cases[j] = cases[j-1] +1;
	  				}
	  				break;
	  			}
	  			else{
	  				if(i==1)
	  					cases[0] = cases[0]+1;
	  				for(int j=1;j<pt_size;j++)
	  					cases[j] = cases[j-1]+1;
	  			}
	  		}
	  	}
	  }
  return output;
}

void CircleFind::sorting(vector<Vec3f> &circles){
  map<pair<int,int>,int> temp;
  for(int i=0;i<circles.size();i++)
    temp.insert(make_pair(make_pair(circles[i][0],circles[i][1]),circles[i][2]));
  int index =0;
  for(auto iter = temp.begin();iter != temp.end();iter++){
    circles[index][0] = iter->first.first;
    circles[index][1] = iter->first.second;
    circles[index][2] = iter->second;
    index++;
  }
}

#endif 