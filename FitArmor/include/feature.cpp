//
// Created by zh on 2023-10-5, 星期四, 16:49.
//

#include<iostream>
#include<opencv2/opencv.hpp>
#include"fit.h"

using namespace std;
using namespace cv;

void ArmorFit::feature() {

    //识别灯条
    FileStorage fs(config, FileStorage::READ);
    if (!fs.isOpened()){
        cout<<"找不到config.xml配置文件!"<<endl;
        return;
    }
    fs["light_min_wh_ratio"] >> light_min_wh_ratio;
    fs["light_max_wh_ratio"] >> light_max_wh_ratio;
    fs["light_min_area"] >> light_min_area;
    fs["light_min_angle"] >> light_min_angle;
    fs["light_max_angle"] >> light_max_angle;
    fs.release();


    vector<vector<Point>> contours;
    vector<my_rotatedrect> armor_light;

    //识别轮廓
    findContours(temp_image,contours,RETR_EXTERNAL,CHAIN_APPROX_NONE);

    //筛选灯条
    for (const auto & contour : contours) {

        if (contour.size() < 5)continue;                       //重要，at least 5 points to fit the ellipse
        my_rotatedrect box;
        to_my_rorec(minAreaRect(contour), box);

        double light_wh_ratio=box.aspectRatio;
        if (light_wh_ratio < light_min_wh_ratio)continue;

        if (light_wh_ratio > light_max_wh_ratio)continue;
        if (box.area < light_min_area)continue;

        if (box.angle > light_max_angle && box.angle < light_min_angle)continue;

        armor_light.push_back(box);
    }

    this->light = armor_light;

}

void ArmorFit::to_my_rorec(const RotatedRect & rect, my_rotatedrect & myrect) {
    Point2f ps[4];
    rect.points(ps);
    int bl_infex;
    float bl_x=100000;
    for (int i = 0; i < 4; ++i) {
        if (ps[i].y>=rect.center.y &&ps[i].x<bl_x){
            bl_x=ps[i].x;
            bl_infex=i;
        }
    }

    for (int i = 0; i < 4; ++i) {
        myrect.points[i]=ps[(bl_infex+i)%4];
    }

    if (bl_infex!=0){
        myrect.width=rect.size.height;
        myrect.height=rect.size.width;
        myrect.angle=rect.angle-90;}
    else{
        myrect.width=rect.size.width;
        myrect.height=rect.size.height;
        myrect.angle=rect.angle;
    }

    myrect.aspectRatio=myrect.width/myrect.height;
    myrect.area=rect.size.area();
    myrect.center=rect.center;
}