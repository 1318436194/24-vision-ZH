//
// Created by zh on 2023-10-5, 星期四, 15:57.
//

#include <iostream>
#include<opencv2/opencv.hpp>
#include "fit.h"

using namespace std;
using namespace cv;

int main(int argc,char ** argv)
{
    ArmorFit armor;

    String src_path,out_path;
    src_path=argc>1?argv[1]:"装甲板.avi";
    out_path=argc==2?argv[2]:"装甲板效果.avi";

    VideoCapture capture(src_path);
    if (!capture.isOpened()){
        cout<<"视频文件"<<src_path<<"不存在！"<<endl;
        return -1;
    }

    auto fps = capture.get(CAP_PROP_FPS);
    Size size((int)capture.get(CAP_PROP_FRAME_WIDTH),
              (int)capture.get( CAP_PROP_FRAME_HEIGHT));
    VideoWriter writer(out_path,0,fps,size, true);
    Mat frame;

    while (capture.read(frame)){
        if (frame.empty())break;

        resize(frame, frame, Size(frame.cols, frame.rows), 0,0,INTER_LINEAR);

        armor.init(frame);
        armor.feature();
        armor.match();
        writer.write(armor.out_image);


        int key = waitKey(int(1000/fps));
        if (key==32)key= waitKey(0);
        if (key==27)break;
    }
    capture.release();
    writer.release();
    cout<<"装甲板识别效果已写入到:"<<out_path<<endl;
    destroyAllWindows();

    return 0;
}