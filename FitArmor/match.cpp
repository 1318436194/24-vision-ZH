//
// Created by zh on 2023-10-5, 星期四, 16:50.
//

#include<iostream>
#include<opencv2/opencv.hpp>
#include"fit.h"

using namespace std;
using namespace cv;

void ArmorFit::match() {
    //读入装甲板筛选参数
    FileStorage fs("config.xml", FileStorage::READ);
    if (!fs.isOpened()){
        cout<<"找不到config.xml配置文件!"<<endl;
        return;
    }
    fs["light_min_ch_ratio"] >> light_min_ch_ratio;
    fs["light_max_ch_ratio"] >> light_max_ch_ratio;
    fs["light_angle_dif"] >> light_angle_dif;
    fs["light_max_cdif_ratio"] >> light_max_cdif_ratio;
    fs["armor_width"]>>armor_width;
    fs["armor_height"]>>armor_height;
    fs.release();

    //配对 找到灯条中点，连线，取连线的中点
    for (unsigned int i = 0; i < light.size(); i++) {
        for (unsigned int j = i+1; j < light.size(); j++) {
            double dis = distance(light[i].center, light[j].center);
            double dif_Y = abs(light[i].center.y - light[j].center.y);
            double hight = (light[i].height + light[j].height)/2;

            if (dis/hight < light_min_ch_ratio)continue;
            if (dis/hight > light_max_ch_ratio)continue;
            if (dif_Y/hight > light_max_cdif_ratio)continue;
            if (abs(light[i].angle-light[j].angle)>light_angle_dif)continue;


            //旋转矩形4条宽的中点对应装甲板4个顶点坐标
            vector<Point2f>m;
            Point2f *p=light[i].points,*q=light[j].points;
            m.emplace_back((p[0]+p[3])/2);
            m.emplace_back((p[1]+p[2])/2);
            m.emplace_back((q[1]+q[2])/2);
            m.emplace_back((q[0]+q[3])/2);

            //绘出装甲板对角线
            line(src_image,m[0],m[2],Scalar(210,240,80),1,8);
            line(src_image,m[1],m[3],Scalar(210,240,80),1,8);

            //画出装甲板中心点
            Point2f center = (light[i].center+light[j].center)/2;
            circle(src_image,center,4,Scalar(50,255,0),-1,8);

            //计算输出装甲板中心距离和位姿
            Mat tVec,rVec;
            double xyz_angle[3];
            double r_angle;
            Mat camera_matrix;
            Mat distortion_coefficients;
            FileStorage params_read("相机内参及畸变参数.yml",FileStorage::READ);
            if(!params_read.isOpened()){
                cout<<"未找到相机参数文件！"<<endl;
                return;
            }
            params_read["camera_matrix"]>>camera_matrix;
            params_read["distortion_coefficients"]>>distortion_coefficients;

            //装甲板世界坐标
            vector<Point3f>points_of_big_armor{
                    Point3f(-armor_width / 2, armor_height / 2, 0),
                    Point3f(-armor_width / 2, -armor_height / 2, 0),
                    Point3f(armor_width / 2, -armor_height / 2, 0),
                    Point3f(armor_width / 2, armor_height / 2, 0)};

            //解算装甲板中心位置和偏转姿态
            solvePnP(points_of_big_armor,m,camera_matrix,distortion_coefficients,rVec,tVec);
            cout<<"装甲板距离:"<<vec_length(tVec)<<"m"<<endl;
            cout<<tVec<<endl;
            cout<<rVec<<endl;
            Mat rMat;
            Rodrigues(rVec,rMat);
            cout<<rMat<<endl;
            Vec3d line(0,0,1);
            //transpose(rMat,rMat);
            cout<<rMat*line<<endl;
            }
        }

    //输出装甲板识别效果图
    this->out_image=src_image;
    imshow("frame", src_image);
}

double ArmorFit::distance(Point2f a, Point2f b) {
    double dis = sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
    return dis;
}

float ArmorFit::vec_length(const vector<float>& vec){
    float squre_sum = 0;
    for (auto c:vec) {
        squre_sum+=c*c;
    }
    return sqrt(squre_sum)/1000;
}
