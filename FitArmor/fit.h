//
// Created by zh on 2023-10-5, 星期四, 16:9.
//
#pragma once

#include <iostream>
#include<opencv2/opencv.hpp>

using namespace std;
using namespace cv;

class ArmorFit{
public:
    Mat out_image;
    void init(const Mat &src);
    void feature();
    void match();


private:
    Mat src_image;
    Mat temp_image;

    struct my_rotatedrect{
        Point2f center;             //旋转矩形中心点
        float angle;                //旋转矩形的宽
        Point2f points[4];          //4个顶点，以左下角为第一个点，顺时针顺序
        float width;                //以点0和点3构成的边为宽
        float height;               //以点0和点1构成的边为高
        float aspectRatio;          //宽/高
        float area;                 //旋转矩形的面积
    };

    float armor_width;
    float armor_height;

    float light_min_area;
    float light_min_angle;
    float light_max_angle;

    float light_min_wh_ratio;
    float light_max_wh_ratio;

    float light_angle_dif;

    float light_min_ch_ratio;
    float light_max_ch_ratio;

    float light_max_cdif_ratio;

    vector<my_rotatedrect> light;

    static void to_my_rorec(const RotatedRect &rect, my_rotatedrect &myrect);

    static double distance(Point2f,Point2f);

    static float vec_length(const vector<float> &vec);
};
