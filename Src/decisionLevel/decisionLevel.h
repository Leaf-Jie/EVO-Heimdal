#ifndef _DECISION_LEVEL_H_
#define _DECISION_LEVEL_H_

#include "camera/streamRetrieve.h"
#include "investigate/investigate.h"
#include <camera/calibration/doubleCameraCalibration.h>
#include <coordinateSolver/ArmorCoordinateSolver.h>
#include <serial/serial.h>

#define ENEMY_COLOR_MODE ENEMY_RED
#define CAR_TYPE CAR_RED
#define TEST_VIDEO  0


class DecisionLevel : public Dahua::Infra::CThread {
public:
    DecisionLevel(std::string caliCameraDataName);

    DecisionLevel(StreamRetrievePtr &RightStreamData, const string &rightSerialNumber, StreamRetrievePtr& LeftStreamData, const string& leftSerialNumber);

    DecisionLevel(StreamRetrievePtr & RightStreamData,const string & rightSerialNumber, SerialPtr & serialData,  StreamRetrievePtr& LeftStreamData, const string& leftSerialNumber);

    bool start() {
        m_isLoop = true;
        return createThread();
    }

    void stop() {
        m_isLoop = false;

    }

    cv::Mat getRightOutputImage() {
        return rightOutImage_;
    }

    cv::Mat getLeftOutputImage() {
        return leftOutImage_;
    }

    double getCalculationFps() const {
        return calculationFps_;
    }

    void setRightOutputImage(cv::Mat *src) {
        rightOutImage_ = src->clone();
    }

    void setLeftOutputImage(cv::Mat* src) {
        leftOutImage_ = src->clone();
    }

    void setOrientationTest(int orientation){
        orientation = orientation;
    }

    bool getCaptureState() const {
        return captureState_;
    }

    struct SampleDataStruct getSampleData() {
        return sampleData_;
    }

    EnemyColor getEnemyColor() {
        return enemyColor_;
    }

    std::vector<bbox_t> getLeftOuts() {
        return leftOuts;
    }

    std::vector<bbox_t> getRightOuts() {
        return rightOuts;
    }

    //内存释放
    void Dump();


private:
    SampleDataStruct sampleData_;

    cv::Point3f _sentry;
    cv::Point3f _bestEnemy;

    bool captureState_ = false;
    bool vehicleLocation3DFlag = false;
    bool m_isLoop;
    bool sendSentryCommand = false;
    double calculationFps_ = 0;
    int _minDistance;
    int orientation = 0;                             //哨兵打击位置
    int lastOrientation = 0;                         //上一次哨兵打击位置
    int sameOrientation = 0;
    int classId_ = 0;
    int seq = 0;                                    //包序号
    int robotId = 0;                                    //ID

    EnemyColor enemyColor_ = ENEMY_COLOR_MODE;
    CarType carType_ = CAR_TYPE;

    std::vector<cv::Point2f> vehicleLocation2D;
    std::vector<cv::Point3f> vehicleLocation3D;
    std::vector<cv::Point3f> _enemyCarList;
    std::vector<ClientMapStruct> _robotPositionData;
    std::vector<bbox_t> leftOuts;
    std::vector<bbox_t> rightOuts;

    cv::Mat rightOutImage_;
    cv::Mat leftOutImage_;

    StreamRetrievePtr rightStreamData_;
    StreamRetrievePtr leftStreamData_;
    Investigate investigate_;
    ArmorCoordinateSolver * ArmorCoordinateSolver_;
    SerialPtr serialData_;

    /**
     * 线程执行体
     */
    void threadProc();

    /**
     * 测试视频线程执行体
     */
    void threadProcUseTestVideo();

    /**
     * 工业相机线程执行体
     */
    void threadProcUseActual();

        /**
     * 雷达站识别流程
     * @param src 图片
     */
    void investigateProcess(const cv::Mat&leftSrc, const cv::Mat& rightSrc);

    /**
     * 获取识别指令数据
     */
    void getDistinguishData();

    /**
     * 完成一次识别流程调用此函数
     */
    void distinguishDataFinishProc();

    /**
     * 给裁判系统发送指令
     */
    void sendDecisionCmd();

    /**
     * 设置样本图片
     */
    void saveSampleImage(Investigate &investigate);

    /**
     * 机器人数据打包
     */
    void pickRobotData();

    /**
    * 计算线程运行速度
    */
    void calFps();

    /**
    * 计算距离最近的敌方车辆
    */
    void searchBestEnemy(std::vector<cv::Point3f> enemyCarList);

    void getOrientation();
};

typedef Dahua::Memory::TSharedPtr<DecisionLevel> DecisionLevelPtr;
#endif
