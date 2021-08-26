#ifndef _SERIAL_H_
#define _SERIAL_H_

#include"tool/RMDefine.h"
#include"CRC.h"
#include "tool/Conf.h"
#include "Infra/Thread.h"
#include "GenICam/Frame.h"
#include "CRC.h"
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <errno.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>
#include <cstdio>
#ifdef _LINUX
#include <unistd.h>
#include <termios.h>
#include <sys/ioctl.h>
#endif

using namespace Dahua::Infra;

//机器人交互信息
struct  RobotDataStruct
{
    ClientMapStruct targetInformation;
    SentryControlStruct sendSentryData;
};

struct JudgeDataStruct{
    EnemyColor enemyColor;

};

class Serial : public CThread{
public:
    Serial();

    bool start();

    void stop();

    /**
     * 裁判系统
     */
    bool judgeSend( std::vector<ClientMapStruct> _RobotPositionData, bool& _sendSentryCommand);


    bool getCalReady() {
        return calReady_;
    }

    void setCalReady(bool iReady) {
        calReady_ = iReady;
    }

    EnemyColor getEnemyColor() {
        return judgeData.enemyColor;
    }

    void setOrientation(uint8_t orientation){
        RobotData.sendSentryData.orientation = orientation;
    }

private:
    RobotDataStruct RobotData;
    Number number;
    DataLenStruct DataLen;
    JudgeDataStruct judgeData;


    std::vector<uchar> _FIFOBuffer;
    std::vector<ClientMapStruct> _RobotPositionDataTest;
    bool calReady_ = false;
    double calculationFps_;
    bool m_isLoop;
    int _isLost = 0;
    int bytes;
    int r_count;                                            //判断是否读取到数据
    int _fd2jud;
    int lastSeq;                                            //上一包序号
    uint8_t _seq;
    uint8_t robot_id;
    uint8_t sentry_id;
    uint8_t header;                                         //头帧位置
    uint8_t sendSentry[19];
    uint8_t sendCoordinate[23];
    uint8_t buf[256] = {0};                                 //存放接收到的数据
    uint8_t bufferDecode[256] = {0};                        //有效数据存储
    uint16_t _sendSentryDataLong = 19;
    uint16_t _sendCoordinateDataLong =23;
    uint16_t  bufLong = 0;                                       //数据长度
    uint16_t cmd_id;                                         //cmd_id 命令码
    bool openSerialFlag = true;

    int test_id = 2;
    float test_x = 4.0f;
    float test_y = 3.0f;
    int test_seq = 0;

    /**
    * 接收裁判系统串口信息
    * @param fd 串口序号
    */
    void paraJudgeReceiver(int& _fd1);

    /**
     * 获得串口命令（解包一部分）
     */
    void preVerityJudgeData();

    /**
     * 计算串口线程每秒运行速度
     */
    void calFps();

    /**
     * 打开串口
     * @param dev_name 串口名称
     * @return 成功标志
     */
    int openPort(const char *dev_name);

    /**
     * 设置裁判系统串口
     * @param fd 串口序号
     * @return 成功标志
     */
    int configureJudgePort(int fd);

    /**
      * 机器人状态数据解包
      * @return 成功标志
      */
    bool unpackRobotStatusData();

     /**
     * 控制哨兵数据打包
     */
    void sendSentryDataPack();

    /**
     * 敌人坐标位置数据打包
     */
    void sendCoordinateDataPack(ClientMapStruct botPosition);

    /**
     * 线程执行体
     */
    void threadProc();

};

typedef Dahua::Memory::TSharedPtr<Serial> SerialPtr;
#endif
