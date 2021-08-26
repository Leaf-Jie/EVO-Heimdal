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
    * �з��������
    */
    cv::Point3f ArmorProcess(const cv::Rect leftBoxes, const cv::Rect rightBoxes);

    /**
     * ������һ֡��ʶ��ģʽ
     * @param distinguishMode ʶ��ģʽ
     */
    void GetResult(const cv::Rect leftBoxes, const cv::Rect rightBoxes);

    /**
     * ���һ�����н���
     */
    void SteroTo3D();

    /**
     * ��ת��ʵ����
     */
    void actualCoordinateSolver();

    /**
    * ����ͼ�����
    */
    void ImgCalibration(const cv::Mat& leftSrc, const cv::Mat& rightSrc);


    /**
    * Ŀ�����
    * @param types ����������
    * @param bboxes ��������
    * @param src ͼ��
    */
    bool runTracker(std::vector<cv::Rect>& bboxes, const cv::Mat& src);

    void initializeTracker(std::vector<cv::Rect>& bboxes, const cv::Mat& src);

    /**
     * ��ȡ�Ƿ�����pitch��yaw��ת�Ƕ�
     * @return
     */
    bool getResetActualCoordinate() const {
        return _resetActualCoordinate;
    }

private:

    cv::Mat _rotationMatrix, _translationMatrix;		//������ͷ�����������ͷ����ת��ƽ�ƾ���
        //��������ͷ���Ե���ת��ƽ�ƾ���
    cv::Mat _rotationMatrixL;
    cv::Mat _rotationMatrixR;
    //���������У׼������ϵ�е�ͶӰ����
    cv::Mat _projectionMatrixL, _projectionMatrixR;
    //���ӳ�����
    cv::Mat _depthMappingMatrix;
    cv::Mat _cameraMatrixL, _cameraMatrixR;
    cv::Mat _distCoeffL, _distCoeffR;

    float _c_x, _c_y;   //������λ�õ�ֵ
    float _f, _b;		//����ͻ���

    cv::Point3f _actualCoordinate = cv::Point3f(0.0f, 0.0f, 0.0f);      //Ŀ�굽������ľ���
    cv::Point3f _cameraCoordinate = cv::Point3f(5358, 2500, -900.0f);      //������״�վ������3ά����
    cv::Point3f _areCoordinate = cv::Point3f(0, 0, 0);                  //Ŀ���ڳ��ص���ά����
    cv::Point2f _leftCameraCenter = cv::Point2f(0.0f, 0.0f);                //�����Ŀ����
    cv::Point2f _rightCameraCenter = cv::Point2f(0.0f, 0.0f);                //�����Ŀ������

    bool _resetActualCoordinate = false;
};

#endif

