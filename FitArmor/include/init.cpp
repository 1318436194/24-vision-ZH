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
    this->out_image = src.clone();

    FileStorage fs(config, FileStorage::READ);
    if (!fs.isOpened()){
        cout<<"找不到config.xml配置文件!"<<endl;
        return;
    }
    fs["light_color"]>> this->light_color;
    fs["thresh"]>> this->thresh;
    fs.release();

    vector<Mat>BGRChannels;
    Mat BlueChannel;
    Mat RedChannel;
    Mat BinaryImage;

    split(this->src_image, BGRChannels);
    BlueChannel = BGRChannels.at(0);
    RedChannel = BGRChannels.at(2);

    //图像二值化
    threshold(light_color=="blue"?BlueChannel:RedChannel, BinaryImage, thresh,255,THRESH_BINARY);

    this->temp_image = BinaryImage;
//    imshow("SrcImage", this->src_image);
//    imshow("BinaryImage",BinaryImage);
}