#ifndef _RM_DEFINE_H_
#define _RM_DEFINE_H_

#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>

//���λ��
typedef enum {
    CAMERA_LEFT = 0,
    CAMERA_RIGHT
}CameraPosition;
//������ɫ
typedef enum {
    ENEMY_RED = 0,
    ENEMY_BLUE
} EnemyColor;
//��������ģʽ
typedef enum {
    SAVE_DISABLE = 0,
    AUTO_SAVE
} SaveSampleMode;
enum EXTREMUM {
    SHORT_SIDE = 0,
    LONG_SIDE
};
//��������
typedef enum {
    CAR_RED = 0,
    CAR_BLUE,
    CAR_UNKNOW,
    WATCHER
} CarType;
//32λ������
typedef union {
    uchar u8_temp[4];
    float float_temp;
    int32_t s32_temp;
    uint32_t u32_temp;
} FormatTrans32Struct;
//16λ������
typedef union {
    uchar u8_temp[2];
    int16_t s16_temp;
    uint16_t u16_temp;
} FormatTrans16Struct;
//Point3f �ṹ��
typedef struct {
    FormatTrans16Struct x;
    FormatTrans16Struct y;
    FormatTrans16Struct z;
} Point3FUnionStruct;
//���������ṹ��
struct SampleDataStruct {
    cv::Mat image;
    bool classifyState = false;
};

//ƫ��λ��
struct DataLenStruct
{
    int sof = 0;
    int data_length = 1;
    int seq = 3;
    int CRC8 = 4;
    int cmd_id = 5;
    int dataheardId = 7;
    int senderid = 9;
    int receiverid = 11;
    int data = 13;
};

struct Number
{
    size_t one = 1;
    size_t two = 2;
    size_t len = 4;
};

//�з������˷�λ
struct ClientMapStruct {
    uint8_t SOF = 0xA5;
    uint16_t data_length = 14;
    uint8_t seq = 0;
    uint8_t CRC8;
    uint16_t cmd_id = 0x0305;
    uint16_t target_robot_ID;
    float target_position_x;
    float target_position_y;
    float toward_angele;
    uint16_t CRC16;
};

//�����ڱ�����
struct SentryControlStruct {
    uint8_t SOF = 0xA5;
    uint16_t data_length = 7;
    uint8_t seq = 0;
    uint8_t CRC8;
    uint16_t cmd_id = 0x0301;
    uint16_t data_cmd_id = 0x0200;
    uint16_t sender_ID;
    uint16_t receiver_ID;
    uint8_t orientation = 0;
    uint16_t CRC16;
};

#endif
