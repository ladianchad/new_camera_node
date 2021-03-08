#ifndef CIRCLE_FIND_H
#define CIRCLE_FIND_H
#include "circle_find.h"
#include <stdlib.h>
#include <string>
CircleFind::CircleFind(string topicname[2],ImgTfFinder::CameraInfo cam_info[2]): it_(nh_)
{
  topic_name[0] = topicname[0];
  topic_name[1] = topicname[1];
  File_Path = "/home/airo/robot_ws/src/camera_node/yolo_data/";
  readindex.open(File_Path+"index.txt");

  ////////// window setting ///////////
  namedWindow("/camera1/usb_cam1_1/image_raw");
  moveWindow("/camera1/usb_cam1_1/image_raw", 0,0);
  resizeWindow("/camera1/usb_cam1_1/image_raw", 720, 405);

  namedWindow("/camera1/usb_cam1_2/image_raw");
  moveWindow("/camera1/usb_cam1_2/image_raw", 720,0);
  resizeWindow("/camera1/usb_cam1_2/image_raw", 720, 405);

  namedWindow("/camera2/usb_cam2_1/image_raw");
  moveWindow("/camera2/usb_cam2_1/image_raw", 0,455);
  resizeWindow("/camera2/usb_cam2_1/image_raw", 720, 405);

  namedWindow("/camera2/usb_cam2_2/image_raw");
  moveWindow("/camera2/usb_cam2_2/image_raw", 720,455);
  resizeWindow("/camera2/usb_cam2_2/image_raw", 720, 405);

  namedWindow("RIGHT HIGH SCOREhigh score");
  moveWindow("RIGHT HIGH SCOREhigh score", 1440,300);
  resizeWindow("RIGHT HIGH SCOREhigh score",540, 250);

  namedWindow("LEFT HIGH SCOREhigh score");
  moveWindow("LEFT HIGH SCOREhigh score", 1440,0);
  resizeWindow("LEFT HIGH SCOREhigh score", 540, 250);

  namedWindow("yolo_left");
  moveWindow("yolo_left", 1440,550);
  resizeWindow("yolo_left", 540, 250);

  namedWindow("yolo_right");
  moveWindow("yolo_right", 1440,850);
  resizeWindow("yolo_right", 540, 250);

  if(readindex.is_open()){
  	string txt;
    getline(readindex,txt);
    img_index = stoi(txt);
    readindex.close();
    cout<<"initial index = "<<img_index<<endl;
   }
  else{
    img_index = 0;
    outFile.open(File_Path+"index.txt");
	outFile.write(std::to_string(img_index).c_str(),sizeof(img_index));
	outFile.close();
    cout<<"create"<<endl;
  }
  image_sub_1 = it_.subscribe(topic_name[0], 1 , &CircleFind::imageCb_1, this);
  image_sub_2= it_.subscribe(topic_name[1], 1 , &CircleFind::imageCb_2, this);
  tfFinder = new ImgTfFinder(cam_info[0],cam_info[1],30,0);
  yoloy_client = nh_.serviceClient<assembly_robot_msgs::Hole_poses>("hole_find_server");
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
  	Image_1 = *msg;
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
     Image_2 = *msg;
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
    Canny(input, edge, canny_1,canny_2);
    dilate(edge, edge, cv::Mat());
    GaussianBlur(input, input, Size(7,7), 0);
    dilate(edge, edge, cv::Mat());
    HoughCircles(edge, circles, HOUGH_GRADIENT, 1,50,120, 23, 5, 50);
    resize(edge,edge, cv::Size( 720, 405), 0, 0, CV_INTER_NN );
    imshow( image_name + "edge",edge);
    waitKey(1);
    circles = sorting(circles);
}

bool CircleFind::find_circle_tf(double dst[3],HoleList &src,int hole_num,int find_num,int missing){
	cout<<"LET'S CALL SERIVECE"<<endl;
	set<pair<int,int>> left_yolo,right_yolo;
	cout<<"[LEFT]"<<endl;
	hole_pixel.request.image = Image_1;
	readindex.open(File_Path+"index.txt");
	string txt;
	getline(readindex,txt);
    img_index = stoi(txt);
    readindex.close();
	outFile.open(File_Path+"index.txt");
	imwrite(File_Path+to_string(img_index++)+".png",opencvImage1);
	imwrite(File_Path+to_string(img_index++)+".png",opencvImage2);
	outFile.write(std::to_string(img_index).c_str(),sizeof(img_index));
	outFile.close();
	if(yoloy_client.call(hole_pixel)){
		for(int i=0;i<hole_pixel.response.hole_pose.size();i++){
			int temp_x = hole_pixel.response.hole_pose[i].x;
			int temp_y = hole_pixel.response.hole_pose[i].y;
			bool is_insert = true;
			for(int i=-60;i<60;i++){
				for(int j=-60;j<60;j++){
					if(left_yolo.count(make_pair(temp_x+i,temp_y+j)))
						is_insert = false;
				}
			}
			if(is_insert)
				left_yolo.insert(make_pair(temp_x,temp_y));
		}
	}
	Mat temp1;
	opencvImage1.copyTo(temp1);
	for(auto iter = left_yolo.begin();iter != left_yolo.end();iter++){
		cout<<"x : "<<iter->first<<" y : "<<iter->second<<endl;
		cv::circle(temp1, Point(iter->first,iter->second),20,Scalar(0,126,250),-1);
	}
	resize(temp1,temp1, cv::Size( 540, 250), 0, 0, CV_INTER_NN );
  	imshow("yolo_left",temp1);
  	waitKey(1);
	cout<<"[RIGHT]"<<endl;

	hole_pixel.request.image = Image_2;
	if(yoloy_client.call(hole_pixel)){
		for(int i=0;i<hole_pixel.response.hole_pose.size();i++){
			int temp_x = hole_pixel.response.hole_pose[i].x;
			int temp_y = hole_pixel.response.hole_pose[i].y;
			bool is_insert = true;
			for(int i=-60;i<60;i++){
				for(int j=-60;j<60;j++){
					if(right_yolo.count(make_pair(temp_x+i,temp_y+j)))
						is_insert = false;
				}
			}
			if(is_insert)
				right_yolo.insert(make_pair(temp_x,temp_y));
		}
	}
	Mat temp2;
	opencvImage2.copyTo(temp2);
	for(auto iter = right_yolo.begin();iter != right_yolo.end();iter++){
		cout<<"x : "<<iter->first<<" y : "<<iter->second<<endl;
		cv::circle(temp2, Point(iter->first,iter->second),20,Scalar(0,126,250),-1);
	}
  	resize(temp2,temp2, cv::Size(540, 250), 0, 0, CV_INTER_NN );
  	imshow("yolo_right",temp2);
  	waitKey(1);
	cout<<"YOLO SERIVECE END!"<<endl;

	if(!left_yolo.size() || !right_yolo.size()){
		cout<<"YOLO CAN'T FIND HOLE!!"<<endl;
		return 0;
	}

	vector<Pattern> left_case,right_case;
	left_case = yolo_case(left_yolo,hole_num);
	right_case = yolo_case(right_yolo,hole_num);
	if(!left_case.size() || !right_case.size()){
		cout<<"CAN'T MAKE CASES!!!"<<endl;
		cout<<"LET'S CONSIDER MISSING HOELS"<<endl;
		find_num = 0;
		hole_num -= missing;
	}
	
	left_case = yolo_case(left_yolo,hole_num);
	right_case = yolo_case(right_yolo,hole_num);

	Score L_score = line_matching(left_case,hole_num);
	Score R_score = line_matching(right_case,hole_num);
	if(!L_score.size() || !R_score.size()){
		cout<<"NO MATCHING CIRCLES!!"<<endl;
		return 0;
	}
	Paring score;
	score  = image_matching(L_score ,R_score,hole_num);

	show_scoreImg(opencvImage1,score.begin()->second.first,"LEFT HIGH SCORE" ,hole_num,find_num);
	show_scoreImg(opencvImage2,score.begin()->second.second,"RIGHT HIGH SCORE",hole_num,find_num);
	if(score.size()){
		cout<<"HOLE FIND SUCCESS!"<<endl<<endl;
		int pixel_data[2][2];
		pixel_data[0][0] = score.begin()->second.first[find_num].first;
		pixel_data[0][1] = score.begin()->second.first[find_num].second;
		cout<<"left : "<<pixel_data[0][0]<<" , "<<pixel_data[0][1]<<endl;
		pixel_data[1][0] = score.begin()->second.second[find_num].first;
		pixel_data[1][1] = score.begin()->second.second[find_num].second;
		cout<<"right : "<<pixel_data[1][0]<<" , "<<pixel_data[1][1]<<endl;
		tfFinder->TransTo3D(dst,pixel_data[0],pixel_data[1]);
		cout<<"[ TF ]"<<endl;
		cout<<"x : "<<dst[0]<<" y : "<<dst[1]<<endl;
		int min_err =  (abs(dst[0])<0.5) &&(abs(dst[1])<0.5);
    	if(min_err)
            return true;
          else
            return false;
	}
	else{
		cout<<"NO MATCHING CIRCLES!!"<<endl;
		return false;
	}
}
vector<Pattern> CircleFind::yolo_case(set<pair<int,int>> &src,int pt_size){
	vector<int> cases;
	vector<Pattern> output;
	vector<pair<int,int>> temp_circle;
	for(auto iter =src.begin();iter != src.end();iter++)
		temp_circle.push_back(*iter);
	int n = temp_circle.size();
	for(int i=0;i<pt_size;i++){
		cases.push_back(i);
	}
	if(n<pt_size)
		return output;
	while(true){
	  	Pattern temp;
	  	for(int i=0;i<pt_size;i++){
	  		temp.push_back(make_pair(temp_circle[cases[i]].first,temp_circle[cases[i]].second));
	  	}
	  	output.push_back(temp);
	  	cout<<"[";
	  	for(int i=0;i<cases.size()-1;i++)
	  		cout<<" "<<cases[i];
	  	cout<<" "<<cases[cases.size()-1]<<" ]"<<endl;
	  	int end =0;
	  	for(int i=0;i<cases.size();i++){
	  		if(cases[i] == n - cases.size() +i)
	  			end ++;
	  	}
	  	if(end == cases.size())
	  		break;
	  	cases[pt_size-1]++;
	  	if(cases[pt_size-1] > n-1){
	  		for(int i=pt_size-1;i>0;i--){
	  			if(i==1){
	  				cases[0] = cases[0]+1;
	  				for(int j=1;j<pt_size;j++)
	  					cases[j] = cases[j-1]+1;
	  			}
	  			else{
	  				cases[i]++;
		  			if(cases[i] < n-1){
		  				for(int j=i+1;j<pt_size;j++){
		  					cases[j] = cases[j-1] +1;
		  				}
		  				break;
		  			}
	  			}
	  		}
	  	}
	  }
  return output;
}
void CircleFind::show_scoreImg(Mat &img,Pattern &src,string name,int hole_num,int find){
	Mat temp;
	img.copyTo(temp);
	int count =0;
	for(int i=0;i<hole_num;i++){
		if(i == find)
			cv::circle(temp, Point(src[i].first,src[i].second),20,Scalar(255,0,0),-1);
		else
			cv::circle(temp, Point(src[i].first,src[i].second),20,Scalar(0,126,250),-1);
	}
	resize(temp,temp, cv::Size( 540, 250), 0, 0, CV_INTER_NN );
	imshow(name+"high score",temp);
	waitKey(1);
}
Paring CircleFind::Y_matching(Paring &src,int pt_size){
	Paring output;
	for(auto iter = src.begin();iter != src.end();iter++){
		double err =0;
		for(int i=1;i<pt_size;i++){
			double dx = abs(iter->second.first[i].first - iter->second.first[i-1].first) + abs(iter->second.second[i].first - iter->second.second[i-1].first); 
			double dy = abs(iter->second.first[i].second - iter->second.first[i-1].second) + abs(iter->second.second[i].second - iter->second.second[i-1].second); 
			err += dy;
		}
		output.insert(make_pair(err,iter->second));
	}
	return output;
}
Paring CircleFind::diff_matching(Paring &src,int pt_size){
	Paring output;
	for(auto iter = src.begin();iter != src.end();iter++){
		double err =0;
		vector<double> dy_score;
		double mean =0;
		for(int i=0;i<pt_size;i++){
			double dx = abs(iter->second.first[i].first - iter->second.second[i].first);
			double dy = abs(iter->second.first[i].second - iter->second.second[i].second);
			double dist = sqrt(pow(dx,2) + pow(dy,2));
			dy_score.push_back(dist);
			mean = (mean*i +dist)/(i+1);
		}
		for(int i=0;i<dy_score.size();i++)
			err += abs(dy_score[i] - mean);
		if(err < 150)
			output.insert(make_pair(err,iter->second));
	}
	return output;
}
Paring CircleFind::image_matching(Score &dst,Score &src,int pt_size){
	Paring output;
	for(auto dst_it = dst.begin();dst_it != dst.end();dst_it++){
		for(auto src_it = src.begin();src_it != src.end(); src_it++){
			double err =0;
			for(int i=0;i<pt_size;i++){
				double x = abs(1920 - dst_it->second[i].first - src_it->second[i].first)/2;
				double y = abs(dst_it->second[i].second - src_it->second[i].second);
				err += (y + x);
			}
			cout<<err<<endl;
			output.insert(make_pair(err*dst_it->first*src_it->first,make_pair(dst_it->second,src_it->second)));
		}
	}
	return output;
}
Score CircleFind::line_matching(vector<Pattern>&src,int pt_size){
	Score output;
	for(int i=0;i< src.size();i++){
		double pre_dx = src[i][1].first - src[i][0].first;;
		double pre_dy = src[i][1].second - src[i][0].second;
		double err =1;
		for(int k=2;k<pt_size;k++){
			double dx = src[i][k].first - src[i][k-1].first;
			double dy = src[i][k].second - src[i][k-1].second;
			err += (abs(dx - pre_dx)+abs(dy - pre_dy));
		}
		output.insert(make_pair(err,src[i]));
	}
	return output;
} 
void CircleFind::show_circles(Mat &img,vector<Vec3f> &circle,string name){
	Mat temp;
	img.copyTo(temp);
	for(int i=0; i<circle.size();i++){
    	cv::circle(temp, Point(circle[i][0],circle[i][1]),circle[i][2],Scalar(0,126,250),-1);
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

vector<Vec3f> CircleFind::sorting(vector<Vec3f> &circles){
  map<pair<int,int>,Vec3f> temp;
  for(int i=0;i<circles.size();i++){
    temp.insert(make_pair(make_pair(circles[i][0],circles[i][1]),circles[i]));
  }
  vector<Vec3f> output;
  for(auto iter = temp.begin();iter != temp.end();iter++){
    output.push_back(iter->second);
  }
  return output;
}

#endif 