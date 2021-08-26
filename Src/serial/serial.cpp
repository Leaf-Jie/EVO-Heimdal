#include "serial.h"

using namespace std;

Serial::Serial()
        : CThread("serial") {
    setCalReady(false);
    _FIFOBuffer.clear();
}

bool Serial::start() {
    m_isLoop = true;
    return createThread();
}

void Serial::stop() {
    m_isLoop = false;
#ifdef _LINUX
    close(_fd2jud);
#endif
}

void Serial::calFps() {
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

int Serial::openPort(const char *dev_name) {
    int fd = -1; // file description for the serial port
#ifdef _LINUX
    fd = open(dev_name, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd == -1) {
        // if open is unsucessful
        LOG::info("open_port: Unable to open " + LOG::tostring(dev_name));
    } else {
        fcntl(fd, F_SETFL, FNDELAY);
        LOG::info("port is open.");
    }
#endif
    return fd;
}

int Serial::configureJudgePort(int fd) {
#ifdef _LINUX

    struct termios port_settings;               // structure to store the port settings in（波特率）
    cfsetispeed(&port_settings, B115200);       // set baud rates
    cfsetospeed(&port_settings, B115200);
    /* Enable the receiver and set local mode...*/
    port_settings.c_cflag |= (CLOCAL | CREAD);
    /* Set c_cflag options.*/
    port_settings.c_cflag &= ~PARENB;         // set no parity, stop bits, data bits（不进行奇偶校验）
    port_settings.c_cflag &= ~PARODD;
    port_settings.c_cflag &= ~CSTOPB;          //一位停止位
    //port_settings.c_cflag &= ~CSIZE;
    port_settings.c_cflag |= CS8;              //八位数据位
    //port_settings.c_cflag &= ~CRTSCTS;
    port_settings.c_iflag &= ~(IXON | IXOFF | IXANY);
    port_settings.c_iflag &= ~(INLCR | IGNCR | ICRNL);
    port_settings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    port_settings.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP);
    /* Set c_oflag output options */
    port_settings.c_oflag &= ~OPOST;
    /* Set the timeout options */
    port_settings.c_cc[VTIME] = 0;
    port_settings.c_cc[VMIN] = 0;
    tcsetattr(fd, TCSANOW, &port_settings);     // apply the settings to the port
    return fd;

#endif // _LINUX
}

//获取裁判系统串口的数据
void Serial::paraJudgeReceiver(int &_fd1) {
#ifdef _LINUX
    ioctl(_fd1, FIONREAD, &bytes);

    static int noDataNum = 0;
    if(bytes == 0){
        noDataNum++;
    }
    if(bytes > 0)
        noDataNum = 0;
    else if(noDataNum > 1000){
        _isLost = 1;
    }

    r_count = read(_fd1, buf, sizeof(buf));         //读取fd1代表的文件，以sizeof(buf)为读取单位，将读取的数据保存到buf
#endif
}

//初步校验裁判系统串口信息
void Serial::preVerityJudgeData() {
#ifdef _LINUX
    for (int i = 0; i < sizeof(buf); ++i) {
        if (buf[i] == 0xA5){
            header = i;
            bufLong = (buf[i + 1] | (uint16_t) buf[i + 2] << 8);
            bufLong += 9;
            for(int j = 0; j < bufLong; j++){
                if((header + j) > 255) break;
                bufferDecode[j] = buf[header + j];
            }
            if (!CRC::verifyCRC8CheckSum(bufferDecode, 5) && !CRC::verifyCRC16CheckSum(bufferDecode, bufLong))
                continue;
            else {
                    cmd_id = (bufferDecode[5] | (uint16_t) bufferDecode[6] << 8);
                    //把串口数据解包
                    switch (cmd_id) {
                        case 0x0201: unpackRobotStatusData();
                            break;
                        default:
                            break;
                    }

            }
        }
    }
#endif
}

//机器人状态数据解包
bool Serial::unpackRobotStatusData() {
#ifdef _LINUX
    if (lastSeq != (int) bufferDecode[3]) {
        robot_id = bufferDecode[7];
        judgeData.enemyColor = 100 > robot_id ? ENEMY_BLUE : ENEMY_RED;
        sentry_id = 100 > robot_id ? 7 : 107;
        calReady_ = true;
    }
    else
        return false;
    lastSeq = (int)bufferDecode[3];
    memset(buf,'\0',sizeof(buf)); //清空数组
    memset(bufferDecode, '\0', sizeof(bufferDecode)); //清空数组
#endif
    return true;
}

#ifdef _LINUX
bool Serial::judgeSend( std::vector<ClientMapStruct> _RobotPositionData, bool& _sendSentryCommand){
#ifdef _LINUX
    if(openSerialFlag) {
        //发送数据
        if (_sendSentryCommand) {
            sendSentryDataPack();
            _sendSentryDataLong == write(_fd2jud, sendSentry, _sendSentryDataLong);
            _sendSentryCommand = false;
        }

        //发送敌方车辆位置
        int robotDataLong = _RobotPositionData.size();
        for(int i = 0; i < robotDataLong; i++){
            sendCoordinateDataPack(_RobotPositionData[i]);
            _sendCoordinateDataLong == write(_fd2jud, sendCoordinate, _sendCoordinateDataLong);
        }
        return true;
#endif
    }
    return false;
}

#endif

//敌人坐标位置数据打包
void Serial::sendCoordinateDataPack(ClientMapStruct botPosition) {
#ifdef _LINUX
    {
        //RobotData
        RobotData.targetInformation.seq = botPosition.seq;
        RobotData.targetInformation.target_robot_ID = botPosition.target_robot_ID;                 //target_id
        RobotData.targetInformation.target_position_x = botPosition.target_position_x;           //target x
        RobotData.targetInformation.target_position_y = botPosition.target_position_y;          //target y
        RobotData.targetInformation.toward_angele = 60.0f;                                      //target_angle

        memset(sendCoordinate, 0, 23);

        //frame_hearder
        memcpy(&sendCoordinate[DataLen.sof], &(RobotData.targetInformation.SOF), number.one);
        memcpy(&sendCoordinate[DataLen.data_length], &(RobotData.targetInformation.data_length), number.two);
        memcpy(&sendCoordinate[DataLen.seq], &(RobotData.targetInformation.seq), number.one);
        CRC::appendCRC8CheckSum(sendCoordinate, 5);

        //cmd_id
        memcpy(&sendCoordinate[DataLen.cmd_id], &RobotData.targetInformation.cmd_id, number.two);

        //data_frame_header
        memcpy(&sendCoordinate[DataLen.dataheardId], &(RobotData.targetInformation.target_robot_ID), number.two);
        memcpy(&sendCoordinate[DataLen.senderid], &(RobotData.targetInformation.target_position_x), number.len);
        memcpy(&sendCoordinate[13], &(RobotData.targetInformation.target_position_y), number.len);
        memcpy(&sendCoordinate[17], &(RobotData.targetInformation.toward_angele), number.len);

        //frame_tail
        CRC::appendCRC16CheckSum(sendCoordinate, 23);
    }
#endif
}

//控制哨兵数据打包
void Serial::sendSentryDataPack()
{
#ifdef _LINUX
    //data_frame_heard
    RobotData.sendSentryData.sender_ID = robot_id;           //send id
    RobotData.sendSentryData.receiver_ID = sentry_id;        //receive_id

    memset(sendSentry, 0, 19);

    //frame_hearder
    memcpy(&sendSentry[DataLen.sof], &(RobotData.sendSentryData.SOF), number.one);
    memcpy(&sendSentry[DataLen.data_length], &(RobotData.sendSentryData.data_length), number.two);
    memcpy(&sendSentry[DataLen.seq], &(RobotData.sendSentryData.seq), number.one);
    CRC::appendCRC8CheckSum(sendSentry, 5);

    //cmd_id
    memcpy(&sendSentry[DataLen.cmd_id], &RobotData.sendSentryData.cmd_id, number.two);

    //data_frame_header
    memcpy(&sendSentry[DataLen.dataheardId], &(RobotData.sendSentryData.data_cmd_id), number.two);
    memcpy(&sendSentry[DataLen.senderid], &(RobotData.sendSentryData.sender_ID), number.two);
    memcpy(&sendSentry[DataLen.receiverid], &(RobotData.sendSentryData.receiver_ID), number.two);

    //data
    memcpy(&sendSentry[DataLen.data], &(RobotData.sendSentryData.orientation), number.len);

    //frame_tail
    CRC::appendCRC16CheckSum(sendSentry, 19);
#endif
}

void Serial::threadProc() {
#ifdef _LINUX

    if (access("/dev/ttyUSB0", F_OK) == 0) {
        _fd2jud = openPort("/dev/ttyUSB0");
        LOG::info("ttyUSB0");
    }
    else {
        openSerialFlag = false;
        LOG::info("No serial");
        //_isLost = 1;
    }

#endif
    if (openSerialFlag) {
        configureJudgePort(_fd2jud);                //配置裁判系统串口
        while (m_isLoop) {
            //获取裁判系统串口的数据
            paraJudgeReceiver(_fd2jud);
            //初步校验裁判系统串口信息
            preVerityJudgeData();
            calFps();                                                               //计算FPS
#ifdef _LINUX
            usleep(10);                           //挂起100us
#endif
        }

     }
}

