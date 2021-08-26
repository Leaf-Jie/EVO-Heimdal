#include "GenICam/System.h"
#include "camera/streamRetrieve.h"
#include "camera/calibration/cameraCalibration.h"
#include "camera/calibration/doubleCameraCalibration.h"
#include "camera/modifyCamera.h"
#include "decisionLevel/decisionLevel.h"
#include "tool/autoSaveSample.h"
#include <csignal>
#pragma GCC optimize (2)

#ifdef _LINUX

#include "GenICam/StreamSource.h"
#include "serial/serial.h"
#include <unistd.h>
#define KILL_SH_PATH    "../script/Heimdal.sh"

#else


#define KILL_SH_PATH    "../Tools/Falcon.sh"

#endif

using namespace std;

int main() {
    //#ifdef LINUX
    int lostNum = 0;
    LOG::setLevel("debug");
    //#else
       // LOG::setLevel("debug");
    //#endif

        //退出代码触发器
    signal(SIGINT, Util::exitHandler);
    signal(SIGTERM, Util::exitHandler);

    //设置日志存储位置
    FileOperation::createDirectory(SAVE_LOG_PATH);
    string logPath = SAVE_LOG_PATH + to_string(FileOperation::getFileSizeInOrder(SAVE_LOG_PATH, ".log")) + ".log";
    LOG::setDestination(logPath);

    //读取代码设置
    try {
        CodeSet::readCodeSet(CODE_SET_NAME);
        CameraConfigurationFactory::readCameraConfiguration(CAMERA_CONFIGURATION_DATA_NAME);
        InstallDataFactory::readInstallData(INSTALL_DATA_NAME);
    }
    catch (cv::Exception& e) {
        LOG::error(e.msg.substr(0, e.msg.length() - 1));
        //遇到异常则全部使用默认参数
        CodeSet::resetAllConfig();
        CameraConfigurationFactory::resetAllConfig();
        InstallDataFactory::resetAllConfig();
        LOG::error("Catch exception while reading YML, reset all configuration!!");
    }

#if !TEST_VIDEO
    //打开工业相机
    Dahua::GenICam::ICameraPtr leftcameraSptr, rightcameraSptr;
    ModifyCamera::CameraInitFlow cameraMode = ModifyCamera::INIT;
    Dahua::GenICam::CSystem& systemObj = Dahua::GenICam::CSystem::getInstance();;
    Dahua::Infra::TVector<Dahua::GenICam::ICameraPtr> vCameraPtrList;
    bool isDiscoverySuccess = false;
    while (cameraMode != ModifyCamera::START_SUCCESS) {
        switch (cameraMode) {
        case ModifyCamera::INIT:
            isDiscoverySuccess = systemObj.discovery(vCameraPtrList);
            cameraMode = ModifyCamera::DISCOVERY_CAMERA;
            break;
        case ModifyCamera::DISCOVERY_CAMERA:
            if (!isDiscoverySuccess) {
                LOG::error("discovery device fail.");
                cameraMode = ModifyCamera::START_FAIL;
            }
            else if (vCameraPtrList.empty()) {
                LOG::error("no devices.");
#ifdef _LINUX
                lostNum++;
                if (lostNum >= 2) {
                    popen(KILL_SH_PATH, "r");

                }
#endif
                cameraMode = ModifyCamera::START_FAIL;
            }
            else {
                cameraMode = ModifyCamera::START_SUCCESS;
            }
            break;
        case ModifyCamera::START_FAIL:
            cameraMode = ModifyCamera::INIT;
            break;
        }
#ifdef _LINUX
        sleep(1);
#else
        Sleep(1000);
#endif
    }

    // 打印相机基本信息（key, 制造商信息, 型号, 序列号）
    if (vCameraPtrList.size() != 2) {
        LOG::error("can't read two camera!\n");
        exit(0);
    }
    for (std::size_t i = 0; i < vCameraPtrList.size(); i++) {
        leftcameraSptr = vCameraPtrList[i];
        LOG::info("Camera Info :");
        LOG::info("   cameraSptr key           = " + LOG::tostring(leftcameraSptr->getKey()));
        LOG::info("   cameraSptr vendor name   = " + LOG::tostring(leftcameraSptr->getVendorName()));
        LOG::info("   cameraSptr model         = " + LOG::tostring(leftcameraSptr->getModelName()));
        LOG::info("   cameraSptr serial number = " + LOG::tostring(leftcameraSptr->getSerialNumber()));
    }
    //分辨出左右相机(左相机序列号一定要小于右相机）
    if (strcmp(vCameraPtrList[0]->getSerialNumber(), vCameraPtrList[1]->getSerialNumber()) > 0) {
        leftcameraSptr = vCameraPtrList[1];
        rightcameraSptr = vCameraPtrList[0];
    }
    else {
        leftcameraSptr = vCameraPtrList[0];
        rightcameraSptr = vCameraPtrList[1];
    }

    if (leftcameraSptr->isConnected() && rightcameraSptr->isConnected()) {
        LOG::error("Camera is already connected!");
    }

    leftcameraSptr->disConnect();
    while (!leftcameraSptr->connect()) {
        LOG::error("Leftcamera connect camera failed.");
#ifdef _LINUX
        lostNum++;
        if (lostNum >= 10) {
            popen(KILL_SH_PATH, "r");
        }
#endif
    }
    rightcameraSptr->disConnect();
    while (!rightcameraSptr->connect()) {
        LOG::error("Rightcamera connect camera failed.");
#ifdef _LINUX
        lostNum++;
        if (lostNum >= 10) {
            popen(KILL_SH_PATH, "r");
        }
#endif
    }
    LOG::info("Connected to camera!");
    //创建左相机流对象
    Dahua::GenICam::IStreamSourcePtr leftstreamPtr = systemObj.createStreamSource(leftcameraSptr);
    if (nullptr == leftstreamPtr) {
        LOG::error("create leftstream obj  fail.");
        return 0;
    }
    LOG::info("create leftstream obj success!");
    leftstreamPtr->stopGrabbing();
    bool leftIsStartGrabbingSuccess = leftstreamPtr->startGrabbing();
    if (!leftIsStartGrabbingSuccess) {
        LOG::error("StartGrabbing left fail.");
    }
    LOG::info("StartGrabbing left success!");
    //创建右相机流对象
    Dahua::GenICam::IStreamSourcePtr rightstreamPtr = systemObj.createStreamSource(rightcameraSptr);
    if (nullptr == rightstreamPtr) {
        LOG::error("create rightstream obj  fail.");
        return 0;
    }
    LOG::info("create rightstream obj success!");
    rightstreamPtr->stopGrabbing();
    bool rughtisStartGrabbingSuccess = rightstreamPtr->startGrabbing();
    if (!rughtisStartGrabbingSuccess) {
        LOG::error("StartGrabbing right fail.");
    }
    LOG::info("StartGrabbing right success!");

    //相机参数设定
    ModifyCamera::modifyCameraInit(leftcameraSptr);
    ModifyCamera::modifyCameraInit(rightcameraSptr);
    LOG::info("Modify camera success!");
    //初始化拉流左相机线程
    Dahua::Memory::TSharedPtr<StreamRetrieve> leftStreamThreadSptr(new StreamRetrieve(leftstreamPtr));
    if (NULL == leftStreamThreadSptr) {
        LOG::error("create leftstream thread obj failed.");
        return 0;
    }
    LOG::info("create rightstream thread obj success!");
    //开始拉流左相机线程
    leftStreamThreadSptr->start();
    //初始化拉流右相机线程
    Dahua::Memory::TSharedPtr<StreamRetrieve> rightStreamThreadSptr(new StreamRetrieve(rightstreamPtr));
    if (NULL == rightStreamThreadSptr) {
        LOG::error("create rightstream thread obj failed.");
        return 0;
    }
    LOG::info("create leftstream thread obj success!");
    //开始拉流右线程
    rightStreamThreadSptr->start();

#ifdef _LINUX
    //初始化串口线程
    Dahua::Memory::TSharedPtr<Serial> serialSptr(new Serial());
    if (NULL == serialSptr) {
        LOG::info("create serial thread obj failed.");
        return 0;
    }
    //开始串口线程
    serialSptr->start();
    LOG::info("Strat getting image!");
    //初始化计算线程
    Dahua::Memory::TSharedPtr<DecisionLevel> decisionLevelSptr(new DecisionLevel( rightStreamThreadSptr, rightcameraSptr->getSerialNumber(), serialSptr,leftStreamThreadSptr, leftcameraSptr->getSerialNumber()));


#else
    //创建计算线程
    Dahua::Memory::TSharedPtr<DecisionLevel> decisionLevelSptr(new DecisionLevel(rightStreamThreadSptr, rightcameraSptr->getSerialNumber(), leftStreamThreadSptr, leftcameraSptr->getSerialNumber()));
#endif
    if (nullptr == decisionLevelSptr) {
        LOG::error("create stream thread obj failed.");
        return 0;
    }
    AutoSaveSample autoSaveSample(decisionLevelSptr, CodeSet::getUseAutoSaveSampleMode());
    cv::Mat saveImgRight;
    cv::Mat saveImgLeft;

    //保存照片参数设定
    FileOperation::createDirectory(SAVE_IMAGE_PATH_RIGHT);
    uint32_t saveNumberRight = FileOperation::getFileSizeInOrder(SAVE_IMAGE_PATH_RIGHT, ".jpg");
    FileOperation::createDirectory(SAVE_IMAGE_PATH_LEFT);
    uint32_t saveNumberLeft = FileOperation::getFileSizeInOrder(SAVE_IMAGE_PATH_LEFT, ".jpg");
    //保存视频参数设定
    while (!leftStreamThreadSptr->getCalReady()) {
        cout << "left streamThread is not ready" << endl;
    }
    while (!rightStreamThreadSptr->getCalReady()) {
        cout << "right streamThread is not ready" << endl;
    }
    cv::VideoWriter* rightVideoWriter = nullptr;
    cv::VideoWriter* leftVideoWriter = nullptr;
    if (CodeSet::getTakePictureFlag()) {
        FileOperation::createDirectory(SAVE_AVI_PATH_RIGHT);
        FileOperation::createDirectory(SAVE_AVI_PATH_LEFT);
        std::string saveNameRight = SAVE_AVI_PATH_RIGHT + to_string(FileOperation::getFileSizeInOrder(SAVE_AVI_PATH_RIGHT, ".avi")) + ".avi";
        std::string saveNameLeft = SAVE_AVI_PATH_LEFT + to_string(FileOperation::getFileSizeInOrder(SAVE_AVI_PATH_RIGHT, ".avi")) + ".avi";
        cv::Size leftSaveSize = cv::Size(leftStreamThreadSptr->getMatImage().cols, leftStreamThreadSptr->getMatImage().rows);
        cv::Size rightSaveSize = cv::Size(rightStreamThreadSptr->getMatImage().cols, rightStreamThreadSptr->getMatImage().rows);
        rightVideoWriter = new cv::VideoWriter(saveNameRight, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 8, leftSaveSize);
        leftVideoWriter = new cv::VideoWriter(saveNameLeft, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 8, rightSaveSize);
        LOG::info("Save avi to " + saveNameRight);
        LOG::info("Save avi to " + saveNameLeft);
    }
    //开始计算线程
    decisionLevelSptr->start();
    LOG::info("Start decision thread!");
    //开始主循环
    while (true) {
        //自动调整曝光时间
        ModifyCamera::autoModifyExposureTime(leftStreamThreadSptr->getMatImage().clone(), CodeSet::getGrayAverage(), leftcameraSptr);
        ModifyCamera::autoModifyExposureTime(rightStreamThreadSptr->getMatImage().clone(), CodeSet::getGrayAverage(), rightcameraSptr);

        //图片必须准备完毕才能显示
        int keyASCII = 0;
            cv::Mat leftLin = leftStreamThreadSptr->getMatImage();
            cv::Mat rightLin = rightStreamThreadSptr->getMatImage();
            decisionLevelSptr->setLeftOutputImage(&leftLin);
            decisionLevelSptr->setRightOutputImage(&rightLin);

            //加载类别名
            string classesFile = "../TrainData/yolo/radar.names";
            ifstream ifs(classesFile.c_str());
            string line;
            vector<string> classes;						//类别名称
            while (getline(ifs, line)) classes.push_back(line);

            std::vector<bbox_t> rightOuts = decisionLevelSptr->getRightOuts();
            std::vector<bbox_t> leftOuts = decisionLevelSptr->getLeftOuts();
            cv::Scalar boxColor;
            if (decisionLevelSptr->getEnemyColor() == ENEMY_RED)
                boxColor = cv::Scalar(0, 0, 255);
            else
                boxColor = cv::Scalar (255, 0, 0);

            autoSaveSample.saveSample();
#ifdef  _LINUX
            //画出右相机的敌人
            int rightOutLong = rightOuts.size();
            for (int r = 0; r < rightOutLong; r++) {
                //绘制方框线条到图片上
                cv::Rect rightBox = cv::Rect(rightOuts[r]. x,rightOuts[r].y, rightOuts[r].w, rightOuts[r].h);
                std::string label = classes[rightOuts[r].obj_id];
                rectangle(rightLin, rightBox, boxColor,3);
                //矩形上标注
                int baseLine;
                cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
                rightBox.y = (rightBox.y > labelSize.height) ? rightBox.y : labelSize.height;
                cv::putText(rightLin, label, cv::Point(rightBox.x, rightBox.y), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 255));
                rightOuts.clear();
            }
            //画出左相机的敌人
            int leftOutLong = leftOuts.size();
            for (int l = 0; l < leftOutLong; l++) {
                cv::Rect leftBox = cv::Rect(leftOuts[l]. x,leftOuts[l].y, leftOuts[l].w, leftOuts[l].h);
                std::string label = classes[leftOuts[l].obj_id];
                rectangle(leftLin, leftBox, boxColor,3);
                //矩形上标注
                int baseLine;
                cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
                leftBox.y = (leftBox.y > labelSize.height) ? leftBox.y : labelSize.height;
                cv::putText(leftLin, label, cv::Point(leftBox.x, leftBox.y), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 255));
                leftOuts.clear();
            }

            if (CodeSet::getUseDebugGuiFlag()) {
                cv::namedWindow("outRightImage", cv::WINDOW_FULLSCREEN);
                cv::imshow("outRightImage", rightLin);
                cv::namedWindow("outLeftImage", cv::WINDOW_FULLSCREEN);
                cv::imshow("outLeftImage", leftLin);
                //LOG::info("frame FPS is " + to_string(leftStreamThreadSptr->getFrameFps()) + ", calculation FPS is " + to_string(decisionLevelSptr->getCalculationFps()));
            }

#else
            //画出右相机的敌人
            int rightOutLong = rightOuts.size();
            for (int r = 0; r < rightOutLong; r++) {
                //绘制方框线条到图片上
                cv::Rect rightBox = cv::Rect(rightOuts[r].x, rightOuts[r].y, rightOuts[r].w, rightOuts[r].h);
                std::string label = classes[rightOuts[r].obj_id];
                rectangle(rightLin, rightBox, boxColor, 3);
                //矩形上标注
                int baseLine;
                cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
                rightBox.y = (rightBox.y > labelSize.height) ? rightBox.y : labelSize.height;
                cv::putText(rightLin, label, cv::Point(rightBox.x, rightBox.y), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 255));
                rightOuts.clear();
            }
            //画出左相机的敌人
            int leftOutLong = leftOuts.size();
            for (int l = 0; l < leftOutLong; l++) {
                cv::Rect leftBox = cv::Rect(leftOuts[l].x, leftOuts[l].y, leftOuts[l].w, leftOuts[l].h);
                std::string label = classes[leftOuts[l].obj_id];
                rectangle(leftLin, leftBox, boxColor, 3);
                                //矩形上标注
                int baseLine;
                cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
                leftBox.y = (leftBox.y > labelSize.height) ? leftBox.y : labelSize.height;
                cv::putText(leftLin, label, cv::Point(leftBox.x, leftBox.y), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 255));
                leftOuts.clear();
            }

            cv::namedWindow("outRightputImage", cv::WINDOW_FREERATIO);
            cv::imshow("outRightputImage", decisionLevelSptr->getRightOutputImage());
            cv::namedWindow("outLeftputImage", cv::WINDOW_FREERATIO);
            cv::imshow("outLeftputImage", decisionLevelSptr->getLeftOutputImage());
            //LOG::info("rightCamera frame FPS is " + to_string(rightStreamThreadSptr->getFrameFps()) + ", calculation FPS is " + to_string(decisionLevelSptr->getCalculationFps()));
           //LOG::info("leatCamera FPS is " + to_string(leftStreamThreadSptr->getFrameFps()) + ", calculation FPS is " + to_string(decisionLevelSptr->getCalculationFps()));
#endif

#ifdef  _LINUX
        if (serialSptr->getCalReady()) {
        }
#endif
        keyASCII = cv::waitKey(1);
        if (keyASCII == 'a') {                //a键减小增益
            ModifyCamera::modifyCameraGainRaw(leftcameraSptr, -0.1, ADD_VALUE);
            ModifyCamera::modifyCameraGainRaw(rightcameraSptr, -0.1, ADD_VALUE);
        }
        else if (keyASCII == 'd') {           //d键增大增益
            ModifyCamera::modifyCameraGainRaw(leftcameraSptr, 0.1, ADD_VALUE);
            ModifyCamera::modifyCameraGainRaw(rightcameraSptr, 0.1, ADD_VALUE);
        }
        else if (keyASCII == 'q') {           //q键减小伽马
            ModifyCamera::modifyCameraGamma(leftcameraSptr, -0.1, ADD_VALUE);
            ModifyCamera::modifyCameraGamma(rightcameraSptr, -0.1, ADD_VALUE);
        }
        else if (keyASCII == 'e') {           //e键增大伽马
            ModifyCamera::modifyCameraGamma(leftcameraSptr, 0.1, ADD_VALUE);
            ModifyCamera::modifyCameraGamma(rightcameraSptr, 0.1, ADD_VALUE);
        }

        else if (keyASCII == 'w') {           //w键增大曝光时间
            ModifyCamera::modifyCameraExposureTime(leftcameraSptr, 100, ADD_VALUE);
            ModifyCamera::modifyCameraExposureTime(rightcameraSptr, 100, ADD_VALUE);
        }
        else if (keyASCII == 's') {           //s键减小曝光时间
            ModifyCamera::modifyCameraExposureTime(leftcameraSptr, -100, ADD_VALUE);
            ModifyCamera::modifyCameraExposureTime(rightcameraSptr, -100, ADD_VALUE);
        }
        else if (keyASCII == 'c') {         //c键截图
            saveImgRight = rightStreamThreadSptr->getMatImage().clone();
            cv::namedWindow("saveImg_left", 0);
            cv::imshow("saveImg_left", saveImgRight);
            saveImgLeft = leftStreamThreadSptr->getMatImage().clone();
            cv::namedWindow("saveImg_right", 0);
            cv::imshow("saveImg_right", saveImgLeft);
        }
        else if (keyASCII == 'v') {       //v键保存截图
            string imgNameRight = SAVE_IMAGE_PATH_RIGHT + to_string(saveNumberRight) + ".jpg";
            string imgNameLeft = SAVE_IMAGE_PATH_LEFT + to_string(saveNumberLeft) + ".jpg";
            imwrite(imgNameRight, saveImgRight);
            imwrite(imgNameLeft, saveImgLeft);
            LOG::info("save image to " + imgNameRight);
            LOG::info("save image to " + imgNameLeft);
            ++saveNumberRight;
            ++saveNumberLeft;
        }
        else if (keyASCII == 'l') {       //l键标定相机
            CCalibration::cameraCali(CAMERA_LEFT, leftcameraSptr->getSerialNumber());
        }
        else if (keyASCII == 'r') {       //r键标定右相机
            CCalibration::cameraCali(CAMERA_RIGHT, rightcameraSptr->getSerialNumber());
        }
        else if (keyASCII == 'z') {       //z键标定相机
            DCalibration::doubleCameraCali(leftcameraSptr->getSerialNumber(), rightcameraSptr->getSerialNumber());
        }
        else if (keyASCII == 'x') {       //x键从文件读入相机参数
            ModifyCamera::modifyCameraAutoSetFromFile(leftcameraSptr);
            ModifyCamera::modifyCameraAutoSetFromFile(rightcameraSptr);
        }
        else if (keyASCII == 27) break;   //ESC退出
        //orientation测试
        else if (keyASCII == 32) //空格
            decisionLevelSptr->setOrientationTest(0);
        else if (keyASCII == 49)
            decisionLevelSptr->setOrientationTest(1);
        else if (keyASCII == 50)
            decisionLevelSptr->setOrientationTest(2);
        else if (keyASCII == 51)
            decisionLevelSptr->setOrientationTest(3);
        else if (keyASCII == 52)
            decisionLevelSptr->setOrientationTest(4);

     //相机丢失退出
        if (leftStreamThreadSptr->getLostCameraCNT() > 50) {
            LOG::error("Leftcamera  lose.");
            break;
        }
        if (rightStreamThreadSptr->getLostCameraCNT() > 50) {
            LOG::error("Rightcamera  lose.");
            break;
        }
        //保存视频
        if (CodeSet::getTakePictureFlag() && leftStreamThreadSptr->getCalReady()) {
            leftVideoWriter->write(leftStreamThreadSptr->getMatImage());
        }
        if (CodeSet::getTakePictureFlag() && rightStreamThreadSptr->getCalReady()) {
            rightVideoWriter->write(rightStreamThreadSptr->getMatImage());
        }
    }
    //停止计算线程
    decisionLevelSptr->stop();
#ifdef  _LINUX
    //停止串口线程
    serialSptr->stop();

#endif
    //停止拉流线程
    leftStreamThreadSptr->stop();
    rightStreamThreadSptr->stop();
    //停止相机拉流
    leftstreamPtr->stopGrabbing();
    rightstreamPtr->stopGrabbing();
    //断开设备
    if (!leftcameraSptr->disConnect()) {
        LOG::info("disConnect camera failed");
    }
    if (!rightcameraSptr->disConnect()) {
        LOG::info("disConnect camera failed");
    }
    LOG::info("disConnect successfully thread ID : " + to_string(Dahua::Infra::CThread::getCurrentThreadID()));

    //使用视频
#else
    //创建计算线程
    Dahua::Memory::TSharedPtr<DecisionLevel> decisionLevelSptr(new DecisionLevel(DOUBLE_CALI_CAMERA_DATA_NAME));
    // Dahua::Memory::TSharedPtr<DecisionLevel> decisionLevelSptr(new DecisionLevel("undefined"));
     //开始计算线程
    decisionLevelSptr->start();
    AutoSaveSample autoSaveSample(decisionLevelSptr, CodeSet::getUseAutoSaveSampleMode());
    while (true) {
        autoSaveSample.saveSample();
        if (cv::waitKey(1) == 27) break;
    }
#endif
    return 0;
}
