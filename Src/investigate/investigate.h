#ifndef _INVESTIGATE_
#define _INVESTIGATE_
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <string.h>
#include "tool/Conf.h"
#define TRACK_OPTFLOW
#define OPENCV
#define GPU
#include "../include/yolo_v2_class.hpp"

using namespace std;

class Investigate
{
public:
    //构造函数
    Investigate();

    void setSampleCaptureState(bool state) {
        _sampleCaptureState = state;
    }

    bool getSampleCaptureState() const {
        return _sampleCaptureState;
    }

    struct SampleDataStruct getSampleData() {
        return _sampleData;
    }

    //雷达站识别进程
    void process(const cv::Mat& src, std::vector<bbox_t>& correctOuts,bool cameraPosition, EnemyColor enemyColor);

    bool getSentryFoundFlag(){
        return sentryFound;
    }

    void setSentryFoundFlag(bool setSentry){
        sentryFound = setSentry;
    }

    int getSentryId(){
        return sentryId;
    }


private:
    //读取网络模型
    void readModel();

    //提取相关数据
    bool extractingInformation(cv::Mat& frame, int classId, float conf, int left, int top, int right, int bottom);

    void ScreeningBox(vector<bbox_t> outs, bool cameraPosition);

    //背景差法
    void backgroundSubtraction(bbox_t box,cv::Mat back, cv::Rect carROI);

    //使用追踪
    void trackCar(cv::Mat& frame, vector<bbox_t>& outs);

    bool informationScreening(cv::Mat& frame, int classId, float conf, int left, int top, int right, int bottom);

    //网络处理相关
    cv::Mat frame;			                            //每一帧
    cv::Mat leftBack, rightBack;									//从每一帧创建一个4D的blob用于网络输入
    //检测超参数
    queue<cv::Mat> trackOptFlow;
    vector<string> classes;						//类别名称
    vector<bbox_t> oldOuts;
    vector<bbox_t> reasonableOuts;
    bool _sampleCaptureState = false;
    bool firsLeftMat = false;                          //取第一帧图像标志
    bool firsRightMat = false;                       //取第一帧图像标志
    bool sentryFound = false;
    int sentryId;
    float oneWeight = 0.0;

    Detector * detector;
    Tracker_optflow trackerFlow;               //用追踪
    track_kalman_t trackKalman;                //卡尔曼实现追踪
    struct SampleDataStruct _sampleData;
    EnemyColor enemyColor_;
};

#endif
