#ifndef _FILEPATH_H_
#define _FILEPATH_H_

#include "tool/fileOperation.h"
#include "tool/PointUtil.h"

//��������ȡ�ļ�
#define CAMERA_CONFIGURATION_DATA_NAME  "../VisionData/cameraConfigurationData.yml"
#define CODE_SET_NAME        "../VisionData/codeSet.yml"                            //���������ļ���
#define SAVE_AVI_PATH_RIGHT       "../VisionData/AVISave/right/"
#define SAVE_AVI_PATH_LEFT       "../VisionData/AVISave/left/"
#define SAVE_IMAGE_PATH_RIGHT     "../VisionData/imgSave/right/"
#define SAVE_IMAGE_PATH_LEFT     "../VisionData/imgSave/left/"
#define SAVE_LOG_PATH       "../log/"

//��������궨�ļ�
#define CALI_RESULTS_PATH            "../VisionData/cameraCaliData/caliResults/"            //�궨��������ļ��洢�ĵ�ַ
#define REPAIR_IMG_PATH                "../VisionData/cameraCaliData/caliRepairImg/"    //��У��ͼ�洢�ĵ�ַ

#define REPAIR_IMG_NAME                "0.jpg"
#define REPAIR_FINISH_IMG_NAME        "1.jpg"

#define CALI_BIAS_DATA_NAME        "caliBiasData.txt"		//�궨ͼ����ƫ������ļ���

//˫Ŀ�궨
#define LEFT_PATTERN_IMG_PATH       "../VisionData/cameraCaliData/caliPatternLeft/"			//�궨ͼ������洢�ĵ�ַ	
#define RIGHT_PATTERN_IMG_PATH		"../VisionData/cameraCaliData/caliPatternRight/"		//�궨ͼ������洢�ĵ�ַ
#define DOUBLE_CALI_CAMERA_DATA_NAME    "doubleCalibCameraResultData.yml"        //�궨��������ļ���

//�Զ�����
#define AUTO_SAVE_TRUE_SAMPLE_PATH      "../TrainData/autoSaveSample/founded/"
#define AUTO_SAVE_FALSE_SAMPLE_PATH     "../TrainData/autoSaveSample/unfounded/"

#endif