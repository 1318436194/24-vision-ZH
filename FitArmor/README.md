# FitArmor

请将配置文件**相机内参及畸变参数.yml**和**config.xml**，和**装甲板原视频**放在编译出的**可执行程序目录下**再运行程序



## 程序结构



### main.cpp

main.cpp仅含读视频和写视频的主程序，不含功能实现



### fit.h

fit.h头文件声明了装甲板识别类ArmorFit用于装甲板识别，ArmorFit最主要的部分为：

**自定义结构体：my_rotatedrect**

包含了以下成员

```c++
struct my_rotatedrect{
        Point2f center;             //旋转矩形中心点
        float angle;                //旋转矩形的宽
        Point2f points[4];          //4个顶点，以左下角为第一个点，顺时针顺序
        float width;                //以点0和点3构成的边为宽
        float height;               //以点0和点1构成的边为高
        float aspectRatio;          //宽/高
        float area;                 //旋转矩形的面积
    };
```

**功能部分：**

>1. 图像预处理函数init(const Mat &src)
>2. 灯条识别函数feature()
>3. 灯条二次筛选及配对函数match()
>4. 将OpenCV的RotatedRect转化为my_rotatedrect的静态函数to_my_rorec(const RotatedRect &rect, my_rotatedrect &myrect)

以上实现分别放在以下三个cpp文件中



### init.cpp

实现图像预处理函数init(const Mat &src)



### feature.cpp

实现灯条识别函数feature()

以及to_my_rorec(const RotatedRect &rect, my_rotatedrect &myrect)



### match.cpp

实现灯条二次筛选及配对函数match()



### config.xml

包含灯条识别和筛选的参数



### 相机内参及畸变参数.yml

如文件名