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
    //���캯��
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

    //�״�վʶ�����
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
    //��ȡ����ģ��
    void readModel();

    //��ȡ�������
    bool extractingInformation(cv::Mat& frame, int classId, float conf, int left, int top, int right, int bottom);

    void ScreeningBox(vector<bbox_t> outs, bool cameraPosition);

    //�����
    void backgroundSubtraction(bbox_t box,cv::Mat back, cv::Rect carROI);

    //ʹ��׷��
    void trackCar(cv::Mat& frame, vector<bbox_t>& outs);

    bool informationScreening(cv::Mat& frame, int classId, float conf, int left, int top, int right, int bottom);

    //���紦�����
    cv::Mat frame;			                            //ÿһ֡
    cv::Mat leftBack, rightBack;									//��ÿһ֡����һ��4D��blob������������
    //��ⳬ����
    queue<cv::Mat> trackOptFlow;
    vector<string> classes;						//�������
    vector<bbox_t> oldOuts;
    vector<bbox_t> reasonableOuts;
    bool _sampleCaptureState = false;
    bool firsLeftMat = false;                          //ȡ��һ֡ͼ���־
    bool firsRightMat = false;                       //ȡ��һ֡ͼ���־
    bool sentryFound = false;
    int sentryId;
    float oneWeight = 0.0;

    Detector * detector;
    Tracker_optflow trackerFlow;               //��׷��
    track_kalman_t trackKalman;                //������ʵ��׷��
    struct SampleDataStruct _sampleData;
    EnemyColor enemyColor_;
};

#endif
