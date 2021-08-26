#ifndef _ARMOR_COORDINATE_SOLVER_H_
#define _ARMOR_COORDINATE_SOLVER_H_

#include <camera/calibration/doubleCameraCalibration.h>
#include <math.h>
#include "tool/Conf.h"

class ArmorCoordinateSolver {
public:
    ArmorCoordinateSolver(DoubleCameraCalibrationStruct calibrationData);

    ~ArmorCoordinateSolver() {}
    /**
    * 敌方坐标解算
    */
    cv::Point3f ArmorProcess(const cv::Rect leftBoxes, const cv::Rect rightBoxes);

    /**
     * 设置上一帧的识别模式
     * @param distinguishMode 识别模式
     */
    void GetResult(const cv::Rect leftBoxes, const cv::Rect rightBoxes);

    /**
     * 最后一步进行解算
     */
    void SteroTo3D();

    /**
     * 旋转真实坐标
     */
    void actualCoordinateSolver();

    /**
    * 左右图像矫正
    */
    void ImgCalibration(const cv::Mat& leftSrc, const cv::Mat& rightSrc);


    /**
    * 目标跟踪
    * @param types 跟踪器类型
    * @param bboxes 矩形数组
    * @param src 图像
    */
    bool runTracker(std::vector<cv::Rect>& bboxes, const cv::Mat& src);

    void initializeTracker(std::vector<cv::Rect>& bboxes, const cv::Mat& src);

    /**
     * 获取是否重置pitch，yaw旋转角度
     * @return
     */
    bool getResetActualCoordinate() const {
        return _resetActualCoordinate;
    }

private:

    cv::Mat _rotationMatrix, _translationMatrix;		//右摄像头相对于左摄像头的旋转和平移矩阵
        //左右摄像头各自的旋转和平移矩阵
    cv::Mat _rotationMatrixL;
    cv::Mat _rotationMatrixR;
    //左右相机在校准后坐标系中的投影矩阵
    cv::Mat _projectionMatrixL, _projectionMatrixR;
    //深度映射矩阵
    cv::Mat _depthMappingMatrix;
    cv::Mat _cameraMatrixL, _cameraMatrixR;
    cv::Mat _distCoeffL, _distCoeffR;

    float _c_x, _c_y;   //相机相对位置的值
    float _f, _b;		//焦距和基线

    cv::Point3f _actualCoordinate = cv::Point3f(0.0f, 0.0f, 0.0f);      //目标到左相机的距离
    cv::Point3f _cameraCoordinate = cv::Point3f(5358, 2500, -900.0f);      //相机在雷达站基座的3维坐标
    cv::Point3f _areCoordinate = cv::Point3f(0, 0, 0);                  //目标在场地的三维坐标
    cv::Point2f _leftCameraCenter = cv::Point2f(0.0f, 0.0f);                //左相机目标中
    cv::Point2f _rightCameraCenter = cv::Point2f(0.0f, 0.0f);                //右相机目标中心

    bool _resetActualCoordinate = false;
};

#endif

