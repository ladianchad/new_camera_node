#ifndef IMG_TH_H
#define IMG_TH_H
#include <cmath>
#include<iostream>
using namespace std;
class ImgTfFinder{
public:
	struct CameraInfo
	{
		double C_x;
		double C_y;
		double F_x;
		double F_y;	
	};
	ImgTfFinder(CameraInfo cam1,CameraInfo cam2,double cam_dist,double cam_angle);
	void TransTo3D(double dst[3],int cam1_src[2],int cam2_src[2]);
	pair<pair<int,int>,pair<int,int>> reverse_to_pixel(double x,double y, double z);

private:
	double dist;
	double angle;
	CameraInfo cameras[2];
	void Calulate_case1(double dst[3],int u[2],int v[2]);
	void Calulate_case2(double dst[3],int u[2],int v[2]);
};

ImgTfFinder::ImgTfFinder(CameraInfo cam1,CameraInfo cam2,double cam_dist,double cam_angle){
	cameras[0].C_x = cam1.C_x;
	cameras[0].C_y = cam1.C_y;
	cameras[0].F_x = cam1.F_x;
	cameras[0].F_y = cam1.F_y;
	cameras[1].C_x = cam2.C_x;
	cameras[1].C_y = cam2.C_y;
	cameras[1].F_x = cam2.F_x;
	cameras[1].F_y = cam2.F_y;
	dist = cam_dist;
	angle = cam_angle;
}

void ImgTfFinder::Calulate_case1(double dst[3],int u[2],int v[2]){
	double Cal_V1_1 = (u[0] - cameras[0].C_x)/(cameras[0].F_x);
	double Cal_V1_2 = (u[1] - cameras[1].C_x)/(cameras[1].F_x);
	double Cal_V2_1 = (cameras[0].C_y - v[0])/(cameras[0].F_y);
	double Cal_V2_2 = (cameras[1].C_y - v[1])/(cameras[1].F_y);
	double temp = (cos(angle) + Cal_V1_2*sin(angle))/(cos(angle) - Cal_V1_1*sin(angle)) - (Cal_V1_2*cos(angle) - sin(angle))/(Cal_V1_1*cos(angle) +sin(angle));
	double D2 = 2*dist/((Cal_V1_1*cos(angle)+sin(angle))*temp);
	double D1 = (cos(angle) - Cal_V1_2*sin(angle))/(cos(angle)+Cal_V1_1*sin(angle))*D2;
	dst[0] = Cal_V1_1*D1*cos(angle) - dist + D1*sin(angle);
	dst[1] = (Cal_V2_1*D1 + Cal_V2_2*D2)/2;
	dst[2] = D1*cos(angle) - Cal_V1_1*D1*sin(angle);
}

void ImgTfFinder::TransTo3D(double dst[3],int cam1_src[2],int cam2_src[2]){

	int u[2],v[2];
	u[0] = cam1_src[0];
	u[1] = cam2_src[0];
	v[0] = cam1_src[1];
	v[1] = cam2_src[1];
	Calulate_case1(dst,u,v);
	
}

pair<pair<int,int>,pair<int,int>> ImgTfFinder::reverse_to_pixel(double x,double y, double z){
	double d1 = z;
	double d2 = z;
	double Ux1 = (x+30)/z;
	double Ux2 = Ux1 - 2*30/z;
	double u1 = Ux1*cameras[0].F_x + cameras[0].C_x;
	double u2 = Ux2*cameras[1].F_x + cameras[1].C_x;
	double Uy = y/z;
	double v1 = cameras[0].C_y- Uy*cameras[0].F_y;
	double v2 = cameras[0].C_y- Uy*cameras[0].F_y;
	return pair<pair<int,int>,pair<int,int>>(make_pair(u1,v1),make_pair(u2,v2));
}

#endif
