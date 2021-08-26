#include "decisionLevel.h"

using namespace std;

DecisionLevel::DecisionLevel(std::string caliCameraDataName) : CThread("decisionLevel") {
    //相机内参信息,畸变系数矩阵读取
    DoubleCameraCalibrationStruct doubleCalibrationData = DCalibration::readCalibrationData(CALI_RESULTS_PATH + caliCameraDataName);
    DCalibration::printCalibrationData(doubleCalibrationData);
    //双目文件读取
    ArmorCoordinateSolver_ = new ArmorCoordinateSolver(doubleCalibrationData);
    this->m_isLoop = false;

}

DecisionLevel::DecisionLevel(StreamRetrievePtr& RightStreamData, const string& rightSerialNumber, StreamRetrievePtr& LeftStreamData, const string& leftSerialNumber) : DecisionLevel(DOUBLE_CALI_CAMERA_DATA_NAME) {
    this->rightStreamData_ = RightStreamData;
    this->leftStreamData_ = LeftStreamData;

}

DecisionLevel::DecisionLevel(StreamRetrievePtr& RightStreamData, const string& rightSerialNumber, SerialPtr& serialData, StreamRetrievePtr& LeftStreamData, const string& leftSerialNumber) : DecisionLevel(RightStreamData, rightSerialNumber, LeftStreamData, leftSerialNumber) {
    this->serialData_ = serialData;
}


void DecisionLevel::sendDecisionCmd() {
#ifdef _LINUX
    if (orientation != lastOrientation){
        sameOrientation = 0;
        sendSentryCommand = true;
    }
    else{
    sameOrientation++;
    if((sameOrientation > 5) && (orientation != 0)) {
        orientation = 0;
        serialData_->setOrientation(orientation);
        sendSentryCommand = true;
     }
    }
    serialData_->judgeSend(_robotPositionData, sendSentryCommand);
#endif
}

void DecisionLevel::calFps() {
    static int count = 0;
    static double lastTime = (double) cv::getTickCount() / cv::getTickFrequency() * 1000; // ms
    ++count;
    // 取固定帧数为100帧计算一次
    if (count >= 100) {
        double curTime = (double) cv::getTickCount() / cv::getTickFrequency() * 1000;
        calculationFps_ = count / (curTime - lastTime) * 1000;
        lastTime = curTime;
        count = 0;
    }
}

//样本保存
void DecisionLevel::saveSampleImage(Investigate &investigate) {
    if (investigate.getSampleCaptureState()) {
        sampleData_ = investigate_.getSampleData();
        captureState_ = true;
    } else {
        captureState_ = false;
    }
    investigate.setSampleCaptureState(false);
}

//计算距离最近的敌方车辆
void DecisionLevel::searchBestEnemy(std::vector<cv::Point3f> enemyCarList) {
    investigate_.setSentryFoundFlag(false);
    int sentryId = investigate_.getSentryId();
    _sentry = enemyCarList[sentryId];
    _enemyCarList = enemyCarList;
    _minDistance = 0;
    int i = _enemyCarList.size();
    float tempDistance;
    cv::Point3f tempPoint;
    float num;
    for(int index=0; index<i;index++){
        if (enemyColor_ = ENEMY_BLUE) {
            classId_ = leftOuts[i].obj_id;
            if (classId_ == CAR_RED)
                continue;
        }
        else {
            classId_ = leftOuts[i].obj_id;
            if (classId_ == CAR_BLUE)
                continue;
        }
        if (index == sentryId)
            continue;
        tempPoint = _enemyCarList[index];
        tempDistance = sqrt(pow(_sentry.x - tempPoint.x, 2) + pow(_sentry.y - tempPoint.y, 2) + pow(_sentry.z - tempPoint.z, 2));
        if(index == 0){
            _minDistance = tempDistance;
            num = index;
        }
        else if(tempDistance < _minDistance){
            _minDistance = tempDistance;
            num = index;
        }
        _bestEnemy = _enemyCarList[num];
    }
    getOrientation();
}

void DecisionLevel::getOrientation(){
    lastOrientation = orientation;
    if (_bestEnemy.x > _sentry.x)
        if (_bestEnemy.z > _sentry .z)
            orientation = 2;
        else
            orientation = 4;
    else if (_bestEnemy.x < _sentry.x)
        if (_bestEnemy.z > _sentry .z)
            orientation = 1;
        else
            orientation = 3;
    else
        orientation = 0;
    serialData_->setOrientation(orientation);

}

//识别调用
void DecisionLevel::investigateProcess(const cv::Mat& leftSrc, const cv::Mat& rightSrc)
{
        Dump();
        investigate_.process(leftSrc, leftOuts, CAMERA_LEFT, enemyColor_);
        investigate_.process(rightSrc, rightOuts, CAMERA_RIGHT, enemyColor_);

   //样本保存
   saveSampleImage(investigate_);

   //返回坐标
   int leftSize = leftOuts.size(), rightSize = rightOuts.size();
   if(leftSize == 0)
       return;
   else if (leftSize == rightSize) {
       if((leftOuts[0].obj_id == rightOuts[0].obj_id) && (leftOuts[leftSize - 1].obj_id == rightOuts[rightSize - 1].obj_id))
       for (int i = 0; i < rightOuts.size(); i++) {
               vehicleLocation3D.push_back(ArmorCoordinateSolver_->ArmorProcess(cv::Rect(leftOuts[i]. x,leftOuts[i].y, leftOuts[i].w, leftOuts[i].h),
                                                                                cv::Rect(rightOuts[i]. x,rightOuts[i].y, rightOuts[i].w, rightOuts[i].h)));
               vehicleLocation2D.push_back(cv::Point2f ( vehicleLocation3D[i].z/1000 , vehicleLocation3D[i].x/1000));
               if(investigate_.getSentryFoundFlag())
                   vehicleLocation2D.erase(vehicleLocation2D.begin() + investigate_.getSentryId());
       }
   }
}

//获取识别指令数据
void DecisionLevel::getDistinguishData() {
#ifdef _LINUX
    if (serialData_->getCalReady()) {
        enemyColor_ = serialData_->getEnemyColor();
        if (enemyColor_ == ENEMY_BLUE) robotId = 101;
        else robotId = 1;
    } else {

        enemyColor_ = ENEMY_COLOR_MODE;
        carType_ = CAR_TYPE;
    }
#else
    enemyColor_ = ENEMY_COLOR_MODE;
#endif
}

//数据传输刷新
void DecisionLevel::distinguishDataFinishProc() {
    pickRobotData();
    sendDecisionCmd();
    _robotPositionData.clear();
}

void DecisionLevel::pickRobotData() {
    if(seq > 2e+8) seq = 0;
    if(robotId == 5) robotId = 1;
    if(robotId == 105) robotId = 101;
    for (int i = 0; i < vehicleLocation2D.size(); ++i) {
        ClientMapStruct RobotData;
        RobotData.seq = seq++;
        RobotData.target_robot_ID = robotId++;                 //target_id
        RobotData.target_position_x =  vehicleLocation2D[i].x;           //target x
        RobotData.target_position_y = vehicleLocation2D[i].y;          //target y
        _robotPositionData.push_back(RobotData);
    }
}

#ifndef _LINUX

//使用视频用于识别测试
void DecisionLevel::threadProcUseTestVideo() {
    getDistinguishData();
    string leftCameraVideoPath = "E:\\code\\Vision2021\\Heimdal\\VisionData\\AVISave\\left\\17.avi";      //测试视频
    string rightCameraVideoPath = "E:\\code\\Vision2021\\Heimdal\\VisionData\\AVISave\\right\\17.avi";      //测试视频

    bool enemyColor = ENEMY_COLOR_MODE;
    cv::Mat area = cv::imread("..\\VisionData\\area.png");
    cv::transpose(area, area);
    if(enemyColor == ENEMY_BLUE)  flip(area, area, 0);  //rotate 90
    else    flip(area, area, 1);  //rotate 270

    cv::VideoCapture leftCap(leftCameraVideoPath);
    cv::VideoCapture rightCap(rightCameraVideoPath);
    if (!leftCap.isOpened() && !rightCap.isOpened()) {
        //如果视频不能正常打开则返回
        if(!leftCap.isOpened())
            LOG::info("No leftVideo");
        else
            LOG::info("No rightVideo");

        exit(-1);
    }
    cv::Mat leftFrame, rightFrame;
    while (m_isLoop) {
        cv::Mat areaRefresh = area.clone();

        leftCap >> leftFrame;
        rightCap >> rightFrame ;
        if (!leftFrame.empty() && !rightFrame.empty()) {
            investigateProcess(leftFrame,rightFrame);

            cv::namedWindow("leftFrame", cv::WINDOW_FREERATIO);
            cv::namedWindow("rightFrame", cv::WINDOW_FREERATIO);
            cv::imshow("leftFrame", leftFrame);
            cv::imshow("rightFrame", rightFrame);
            
            int keyASCII = cv::waitKey(1);
            if (keyASCII == 27) break;                                                    //ESC退出
        } else {                                                                        //视频放完则退出程序
            leftCap.release();                                                          //视频放完则退出程序
            rightCap.release();
            exit(0);
        }
    }
}

#endif

//调用摄像头进行识别处理
void DecisionLevel::threadProcUseActual() {
    while (m_isLoop) {
        //检测是否有新图片输入
        if (rightStreamData_->getCalReady() && leftStreamData_->getCalReady()) {
            getDistinguishData();
            investigateProcess(leftStreamData_->getMatImage(), rightStreamData_->getMatImage());

            //哨兵控制
            vehicleLocation3DFlag = vehicleLocation3D.size() > 0 ? true : false;
            if (investigate_.getSentryFoundFlag() && vehicleLocation3DFlag)
                searchBestEnemy(vehicleLocation3D);
        }
        calFps();                                                               //计算FPS
        distinguishDataFinishProc();                                            //数据传输刷新
    }
}

void DecisionLevel::Dump() {
    vehicleLocation2D.clear();
    vehicleLocation3D.clear();
}

//决策层进程
void DecisionLevel::threadProc() {
#if TEST_VIDEO
    threadProcUseTestVideo();
#else
    threadProcUseActual();
#endif

}


