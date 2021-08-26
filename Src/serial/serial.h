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

//�����˽�����Ϣ
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
     * ����ϵͳ
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
    int r_count;                                            //�ж��Ƿ��ȡ������
    int _fd2jud;
    int lastSeq;                                            //��һ�����
    uint8_t _seq;
    uint8_t robot_id;
    uint8_t sentry_id;
    uint8_t header;                                         //ͷ֡λ��
    uint8_t sendSentry[19];
    uint8_t sendCoordinate[23];
    uint8_t buf[256] = {0};                                 //��Ž��յ�������
    uint8_t bufferDecode[256] = {0};                        //��Ч���ݴ洢
    uint16_t _sendSentryDataLong = 19;
    uint16_t _sendCoordinateDataLong =23;
    uint16_t  bufLong = 0;                                       //���ݳ���
    uint16_t cmd_id;                                         //cmd_id ������
    bool openSerialFlag = true;

    int test_id = 2;
    float test_x = 4.0f;
    float test_y = 3.0f;
    int test_seq = 0;

    /**
    * ���ղ���ϵͳ������Ϣ
    * @param fd �������
    */
    void paraJudgeReceiver(int& _fd1);

    /**
     * ��ô���������һ���֣�
     */
    void preVerityJudgeData();

    /**
     * ���㴮���߳�ÿ�������ٶ�
     */
    void calFps();

    /**
     * �򿪴���
     * @param dev_name ��������
     * @return �ɹ���־
     */
    int openPort(const char *dev_name);

    /**
     * ���ò���ϵͳ����
     * @param fd �������
     * @return �ɹ���־
     */
    int configureJudgePort(int fd);

    /**
      * ������״̬���ݽ��
      * @return �ɹ���־
      */
    bool unpackRobotStatusData();

     /**
     * �����ڱ����ݴ��
     */
    void sendSentryDataPack();

    /**
     * ��������λ�����ݴ��
     */
    void sendCoordinateDataPack(ClientMapStruct botPosition);

    /**
     * �߳�ִ����
     */
    void threadProc();

};

typedef Dahua::Memory::TSharedPtr<Serial> SerialPtr;
#endif
