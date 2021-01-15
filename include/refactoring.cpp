#ifndef CIRCLE_FIND_H
#define CIRCLE_FIND_H
#include "circle_find.h"
#include <algorithm>
#include <fstream>
#include <stdlib.h>
#include <string>
CircleFind::CircleFind(string topicname[2],ImgTfFinder::CameraInfo cam_info[2]): it_(nh_),minDist(30),minRadius(5),maxRadius(15)
{
  topic_name[0] = topicname[0];
  topic_name[1] = topicname[1];
  ifstream readFile;
  ofstream outFile;
  file_path = "/home/care/robot_ws/src/camera_node/img_data";
  file_path += topic_name[0].substr(0,8);
  cout<<file_path<<endl;
  readFile.open(file_path+".txt");
  string txt;
  if(readFile.is_open()){
    getline(readFile,txt);
    img_index = stoi(txt);
    readFile.close();
    cout<<"initial index = "<<img_index<<endl;
  }
  else{
    img_index = 0;
    outFile.open(file_path+".txt");
    outFile.write(std::to_string(img_index).c_str(),std::to_string(img_index).size());
    outFile.close();
    cout<<"create"<<endl;
  }
  image_sub_1 = it_.subscribe(topic_name[0], 1 , &CircleFind::imageCb_1, this);
  image_sub_2= it_.subscribe(topic_name[1], 1 , &CircleFind::imageCb_2, this);
  //test_sub1 = nh_.subscribe("/camera1/usb_cam1/image_raw", 1 , &CircleFind::imageCb_1, this);
  //test_sub2 = nh_.subscribe("/camera2/usb_cam2/image_raw", 1 , &CircleFind::imageCb_2, this);
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
    GaussianBlur(input, input, blur_size, 0);
    dilate(input, input, cv::Mat());
    erode(input, input, cv::Mat());
    erode(input, input, cv::Mat());
    dilate(input, input, cv::Mat());
    Canny(input, edge, canny_1,canny_2);
    HoughCircles(edge, circles, HOUGH_GRADIENT, 1,50,120, 23, 5, 50);
    sorting(circles);
}

bool CircleFind::find_circle_tf(double dst[3],int hole_num,int find_num,vector<pair<int,int>> &L_boundary,vector<pair<int,int>> &R_boundary){
  circle_t(opencvImage1,circles[0],Size(9,9),50,60,150,topic_name[0] +"/circle_1");
  circle_t(opencvImage2,circles[1],Size(9,9),50,60,150,topic_name[1] +"/circle_2");
  if(!circles[0].size() || !circles[1].size())
    return false;
  Mat test1; 
  opencvImage1.copyTo(test1);
  Mat test2;
  opencvImage2.copyTo(test2);

  circle(test1, Point(L_boundary[find_num].first,L_boundary[find_num].second),15,Scalar(255,255,255),2);
  circle(test2, Point(R_boundary[find_num].first,R_boundary[find_num].second),15,Scalar(255,255,255),2);
  cout<<"===================="<<endl<<"LEFT"<<endl<<endl;
  for(int i=0; i<circles[0].size();i++){
    circle(test1, Point(circles[0][i][0],circles[0][i][1]),circles[0][i][2],Scalar(255,0,0),2);
    cout<<"circle "<<i+1<<endl;
    cout<<"x : "<<circles[0][i][0]<<"\ty: "<<circles[0][i][1]<<"\tz: "<<circles[0][i][2]<<endl;
  }
  cout<<endl<<"RIGHT"<<endl<<endl;
  for(int i=0; i<circles[1].size();i++) {
    circle(test2, Point(circles[1][i][0],circles[1][i][1]),circles[1][i][2],Scalar(255,0,0),2);
    cout<<"circle "<<i+1<<endl;
    cout<<"x : "<<circles[1][i][0]<<"\ty: "<<circles[1][i][1]<<"\tz: "<<circles[1][i][2]<<endl;
  }
  data_save();
  cout<<"left_found : "<<circles[0].size()<<endl;
  cout<<"right_found : "<<circles[1].size()<<endl;
  
  bool is_one_hole = false;
  if(L_boundary.size()==1 || R_boundary.size()==1)
  	is_one_hole = true;
  if(!is_one_hole){
    Pattern left_pattern,right_pattern;
    left_pattern = make_pattern(L_boundary);
    right_pattern = make_pattern(R_boundary);
    if(circles[0].size()<left_pattern.size()+1 || circles[1].size()<right_pattern.size()+1)
      return 0;
    cout<<"LEFT"<<endl;
    for(int i=0;i<left_pattern.size();i++){
      cout<<"dx = "<<left_pattern[i].first<<" dy = "<<left_pattern[i].second<<endl;
    }
    cout<<endl;
    cout<<"RIGHT"<<endl;
    for(int i=0;i<right_pattern.size();i++){
      cout<<"dx = "<<right_pattern[i].first<<" dy = "<<right_pattern[i].second<<endl;
    }

    vector<Pattern> left_case,right_case;
    left_case = find_case(circles[0],left_pattern);
    right_case = find_case(circles[1],right_pattern);

    if(!left_case.size() || !right_case.size())
    	return 0;
    Score left_score,right_score;
    left_score = pattern_accuracy_check(left_case,left_pattern);
    right_score = pattern_accuracy_check(right_case,right_pattern);
    int count =0;
    for(auto iter = left_score.begin();iter != left_score.end();iter++){
    	for(int i=0;i<iter->second.size();i++){
    		circle(test1, Point(iter->second[i].first,iter->second[i].second),15,Scalar(0,0,255/iter->first),2);
    		cout<<"x : "<<iter->second[i].first<<"y: "<<iter->second[i].second<<"\t\t";
    	}
    	cout<<endl;
    	cout<<"\taccuracy : "<<iter->first<<endl;
    	if(count++ >3)
    		break;
    }
    cout<<endl<<endl;
    count =0;
    for(auto iter = right_score.begin();iter != right_score.end();iter++){
    	for(int i=0;i<iter->second.size();i++){
    		circle(test2, Point(iter->second[i].first,iter->second[i].second),15,Scalar(0,0,255/iter->first),2);
    		cout<<"x : "<<iter->second[i].first<<" y: "<<iter->second[i].second<<"\t\t";
    	}
    	cout<<endl;
    	cout<<"\taccuracy : "<<iter->first<<endl;
    	if(count++ >3)
    		break;
    }
    int pixel_data[2][2];
    double temp_output[3];
    auto left_appro = left_score.begin();
    auto right_appro = right_score.begin();
    for(int i=0;i<left_appro->second.size();i++){
    	circle(test1, Point(left_appro->second[i].first,left_appro->second[i].second),20,Scalar(0,255,0),2);
    }
    for(int i=0;i<right_appro->second.size();i++){
    	circle(test2, Point(right_appro->second[i].first,right_appro->second[i].second),20,Scalar(0,255,0),2);
    }
    pixel_data[0][0] = left_appro->second[find_num].first;
    pixel_data[0][1] = left_appro->second[find_num].second;
    pixel_data[1][0] = right_appro->second[find_num].first;
    pixel_data[1][1] = right_appro->second[find_num].second;
    resize(test1,test1, cv::Size( 720, 405), 0, 0, CV_INTER_NN );
    resize(test2,test2, cv::Size( 720, 405), 0, 0, CV_INTER_NN );
    imshow(topic_name[0]+"circle_mark_left",test1);
    waitKey(1);
    imshow(topic_name[1]+"circle_mark_right",test2);
    waitKey(1);

    tfFinder->TransTo3D(dst,pixel_data[0],pixel_data[1]);
    int min_err =  (abs(dst[0])<0.5) &&(abs(dst[1])<0.5);
    if(min_err)
            return true;
          else
            return false;
  }
  else{
    Pattern new_pattern;
    new_pattern = make_pattern(L_boundary,R_boundary);
    cout<<"new_pattern"<<endl;
    for(int i=0;i<new_pattern.size();i++){
      cout<<"dx = "<<new_pattern[i].first<<" dy = "<<new_pattern[i].second<<endl;
    }
    vector<Pattern> L_list,R_list;
    for(int i=0;i<circles[0].size();i++){
      Pattern temp;
      temp.push_back(make_pair(circles[0][i][0],circles[0][i][1]));
      L_list.push_back(temp);
    }
    for(int i=0;i<circles[1].size();i++){
      Pattern temp;
      temp.push_back(make_pair(circles[1][i][0],circles[1][i][1]));
      R_list.push_back(temp);
    }
    Score left_score,right_score;
    left_score = pattern_accuracy_check(L_list,R_list, new_pattern);
    right_score = pattern_accuracy_check(R_list,L_list, new_pattern);
    int count =0;
    for(auto iter = left_score.begin();iter != left_score.end();iter++){
      for(int i=0;i<iter->second.size();i++){
        circle(test1, Point(iter->second[i].first,iter->second[i].second),15,Scalar(0,0,255/iter->first),2);
        cout<<"x : "<<iter->second[i].first<<"y: "<<iter->second[i].second<<"\t\t";
      }
      cout<<endl;
      cout<<"\taccuracy : "<<iter->first<<endl;
      if(count++ >3)
        break;
    }
    cout<<endl<<endl;
    count =0;
    for(auto iter = right_score.begin();iter != right_score.end();iter++){
      for(int i=0;i<iter->second.size();i++){
        circle(test2, Point(iter->second[i].first,iter->second[i].second),15,Scalar(0,0,255/iter->first),2);
        cout<<"x : "<<iter->second[i].first<<" y: "<<iter->second[i].second<<"\t\t";
      }
      cout<<endl;
      cout<<"\taccuracy : "<<iter->first<<endl;
      if(count++ >3)
        break;
    }
    int pixel_data[2][2];
    double temp_output[3];
    auto left_appro = left_score.begin();
    auto right_appro = right_score.begin();
    for(int i=0;i<left_appro->second.size();i++){
      circle(test1, Point(left_appro->second[i].first,left_appro->second[i].second),20,Scalar(0,255,0),2);
    }
    for(int i=0;i<right_appro->second.size();i++){
      circle(test2, Point(right_appro->second[i].first,right_appro->second[i].second),20,Scalar(0,255,0),2);
    }
    pixel_data[0][0] = left_appro->second[find_num].first;
    pixel_data[0][1] = left_appro->second[find_num].second;
    pixel_data[1][0] = right_appro->second[find_num].first;
    pixel_data[1][1] = right_appro->second[find_num].second;
    resize(test1,test1, cv::Size( 720, 405), 0, 0, CV_INTER_NN );
    resize(test2,test2, cv::Size( 720, 405), 0, 0, CV_INTER_NN );
    imshow(topic_name[0]+"circle_mark_left",test1);
    waitKey(1);
    imshow(topic_name[1]+"circle_mark_right",test2);
    waitKey(1);

    tfFinder->TransTo3D(dst,pixel_data[0],pixel_data[1]);
    int min_err =  (abs(dst[0])<0.5) &&(abs(dst[1])<0.5);
    if(min_err)
            return true;
          else
            return false;
  }
}

Score CircleFind::pattern_accuracy_check(vector<Pattern> &list, Pattern &pattern){
  Score map;
  if(!pattern.size())
    return map;
  for(int i = 0;i<list.size();i++){
  	double score = 0;
  	double score_x = 0;
  	double score_y = 0;
    for(int j=1;j<list[i].size();j++){
    	score_x += abs((list[i][j].first - list[i][j-1].first- pattern[j-1].first)/(double)(pattern[j-1].first+1))*100;
    	score_y += abs(((list[i][j].second - list[i][j-1].second) - pattern[j-1].second)/(double)(pattern[j-1].second+1))*1.5;
    }
    score_x /= pattern.size();
    score_y /= pattern.size();
    score = sqrt(pow(score_x,2)+pow(score_y,2))/100;

    map.insert(make_pair(score,list[i]));
  }
  return map;
}
Score CircleFind::pattern_accuracy_check(vector<Pattern> &dst,vector<Pattern> &src, Pattern &pattern){
  Score map;
  if(!pattern.size())
    return map;
  for(int i =0;i<dst.size();i++){
    double min_err_score = 100000;
    for(int j=0;j<src.size();j++){
      double score = 0;
      double score_x = 0;
      double score_y = 0;
      for(int k =0;k<dst[i].size();k++){
        score_x += (abs(abs(dst[i][k].first - src[j][k].first) - pattern[k].first)/(double)pattern[k].first)*100;
        //score_y += (abs(abs(dst[i][k].second - src[j][k].second) - pattern[k].second)/(double)(pattern[k].second+1))*0.7;
      }
      score_x /= pattern.size();
      score_y /= pattern.size();
      score = sqrt(pow(score_x,2)+pow(score_y,2))/100;
      if(min_err_score > score)
        min_err_score = score;
    }
    map.insert(make_pair(min_err_score,dst[i]));
  }
}
vector<Pattern> CircleFind::find_case(vector<Vec3f> &circle, Pattern &pattern){
  cout<<"CIRCLE : "<<circle.size()<<endl;
  cout<<"pattern : "<<pattern.size()+1<<endl;
  int case_size = pattern.size()+1;
  int case_count = 0;
  vector<Pattern> output;
  vector<int> case_index;
  for(int i=0;i<case_size;i++)
    case_index.push_back(i);
  while(true){
    Pattern  temp;
    for(int j=0;j<case_size;j++)
      temp.push_back(make_pair(circle[case_index[j]][0],circle[case_index[j]][1]));
    output.push_back(temp);
    int end = 0;
  	for(int i=0;i<case_size;i++){
  		if(case_index[i] == circle.size() - case_index.size() + i)
  			end++;
  	}
  	if(end == case_size)
  		break;
    case_index[pattern.size()]++;
    if(case_index[pattern.size()]>circle.size()-1){
      for(int j=pattern.size()-1;j>0;j--){ 
        case_index[j]++;
        if(case_index[j]<circle.size()-1){
          for(int k=j+1;k<case_size;k++){
            case_index[k] = case_index[k-1]+1;
          }
          break;
        }
        else{
          if(j == 1){
            case_index[0]++;
            for(int k=1;k<case_size;k++){
              case_index[k] = case_index[k-1]+1;
            }
          }
        }
      }
    }
  }
  return output;
}

Pattern CircleFind::make_pattern(Pattern &input){
  Pattern output;
  if(input.size() ==1){
    return input;
  }
  for(int i =1;i<input.size();i++)
    output.push_back(make_pair(input[i].first - input[i-1].first , input[i].second - input[i-1].second));
  return output;
}

Pattern CircleFind::make_pattern(Pattern &input1,Pattern &input2){
  Pattern output;
  output.push_back(make_pair(abs(input2[0].first - input1[0].first),abs(input2[0].second-input1[0].second)));
  return output;
}

void CircleFind::data_save(){
  imwrite("/home/care/robot_ws/src/camera_node/yolo_data/camera"+std::to_string(img_index++)+".jpg",opencvImage1);
  imwrite("/home/care/robot_ws/src/camera_node/yolo_data/camera"+std::to_string(img_index--)+".jpg",opencvImage2);
  for(int i=0; i<circles[0].size();i++){
    Rect bounds(0,0,opencvImage1.cols,opencvImage1.rows);
    Rect r(circles[0][i][0] -circles[0][i][2]*1.5, circles[0][i][1] -circles[0][i][2]*1.5, circles[0][i][2]*3, circles[0][i][2]*3);
    Mat temp = opencvImage1(r&bounds);
    resize( temp,temp, cv::Size( 28, 28), 0, 0, CV_INTER_NN );
    imwrite(file_path+"_"+std::to_string(img_index++)+".jpg",temp);
  }
  for(int i=0; i<circles[1].size();i++) {
    Rect bounds(0,0,opencvImage2.cols,opencvImage2.rows);
    Rect r(circles[1][i][0] -circles[1][i][2]*1.5, circles[1][i][1] -circles[1][i][2]*1.5, circles[1][i][2]*3, circles[1][i][2]*3);
    Mat temp = opencvImage2(r&bounds);
    resize( temp,temp, cv::Size( 28, 28), 0, 0, CV_INTER_NN );
    imwrite(file_path+"_"+std::to_string(img_index++)+".jpg",temp);
  }
  ofstream outFile;
  outFile.open(file_path+".txt");
  outFile.write(std::to_string(img_index).c_str(),std::to_string(img_index).size());
  outFile.close();
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