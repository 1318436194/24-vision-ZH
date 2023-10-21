//
// Created by zh on 2023-10-5, 星期四, 16:45.
//

#include <iostream>
#include<opencv2/opencv.hpp>
#include "fit.h"

using namespace std;
using namespace cv;

void ArmorFit::init(const Mat &src) {
    this->src_image = src;

    String light_color;
    FileStorage fs("config.xml", FileStorage::READ);
    if (!fs.isOpened()){
        cout<<"找不到config.xml配置文件!"<<endl;
        return;
    }
    fs["light_color"]>>light_color;
    fs.release();

    vector<Mat>BGRChannels;
    Mat BlueChannel;
    Mat RedChannel;
    Mat BinaryImage;

    split(src_image, BGRChannels);
    BlueChannel = BGRChannels.at(0);
    RedChannel = BGRChannels.at(2);

    //图像二值化
    threshold(light_color=="blue"?BlueChannel:RedChannel, BinaryImage, 200,255,THRESH_BINARY);

    this->temp_image = BinaryImage;
    imshow("SrcImage",src);
    imshow("BinaryImage",BinaryImage);
}