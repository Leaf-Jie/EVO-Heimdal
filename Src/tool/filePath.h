#ifndef _FILEPATH_H_
#define _FILEPATH_H_

#include "tool/fileOperation.h"
#include "tool/PointUtil.h"

//主函数读取文件
#define CAMERA_CONFIGURATION_DATA_NAME  "../VisionData/cameraConfigurationData.yml"
#define CODE_SET_NAME        "../VisionData/codeSet.yml"                            //代码设置文件名
#define SAVE_AVI_PATH_RIGHT       "../VisionData/AVISave/right/"
#define SAVE_AVI_PATH_LEFT       "../VisionData/AVISave/left/"
#define SAVE_IMAGE_PATH_RIGHT     "../VisionData/imgSave/right/"
#define SAVE_IMAGE_PATH_LEFT     "../VisionData/imgSave/left/"
#define SAVE_LOG_PATH       "../log/"

//单边相机标定文件
#define CALI_RESULTS_PATH            "../VisionData/cameraCaliData/caliResults/"            //标定相机参数文件存储的地址
#define REPAIR_IMG_PATH                "../VisionData/cameraCaliData/caliRepairImg/"    //需校正图存储的地址

#define REPAIR_IMG_NAME                "0.jpg"
#define REPAIR_FINISH_IMG_NAME        "1.jpg"

#define CALI_BIAS_DATA_NAME        "caliBiasData.txt"		//标定图像素偏差参数文件名

//双目标定
#define LEFT_PATTERN_IMG_PATH       "../VisionData/cameraCaliData/caliPatternLeft/"			//标定图左相机存储的地址	
#define RIGHT_PATTERN_IMG_PATH		"../VisionData/cameraCaliData/caliPatternRight/"		//标定图右相机存储的地址
#define DOUBLE_CALI_CAMERA_DATA_NAME    "doubleCalibCameraResultData.yml"        //标定相机参数文件名

//自动保存
#define AUTO_SAVE_TRUE_SAMPLE_PATH      "../TrainData/autoSaveSample/founded/"
#define AUTO_SAVE_FALSE_SAMPLE_PATH     "../TrainData/autoSaveSample/unfounded/"

#endif