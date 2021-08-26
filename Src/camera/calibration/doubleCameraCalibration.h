#ifndef _DOUBLE_CAMERA_CALIBRATION_H_
#define _DOUBLE_CAMERA_CALIBRATION_H_

#include "camera/calibration/cameraCalibration.h"

struct DoubleCameraCalibrationStruct {
	float cameraMatrixL0_0, cameraMatrixL0_2, cameraMatrixL1_1, cameraMatrixL1_2;                //����ڲ���Ϣ
	float distCoeffL0_0, distCoeffL1_0, distCoeffL2_0, distCoeffL3_0, distCoeffL4_0;
	float cameraMatrixR0_0, cameraMatrixR0_2, cameraMatrixR1_1, cameraMatrixR1_2;                //����ڲ���Ϣ
	float distCoeffR0_0, distCoeffR1_0, distCoeffR2_0, distCoeffR3_0, distCoeffR4_0;
	float R0_0, R0_1, R0_2, R1_0, R1_1, R1_2, R2_0, R2_1, R2_2;                //����ڲ���Ϣ
	float T0_0, T0_1, T0_2;                //����ڲ���Ϣ
	float Rl0_0, Rl0_1, Rl0_2, Rl1_0, Rl1_1, Rl1_2, Rl2_0, Rl2_1, Rl2_2;                //����ڲ���Ϣ
	float Rr0_0, Rr0_1, Rr0_2, Rr1_0, Rr1_1, Rr1_2, Rr2_0, Rr2_1, Rr2_2;                //����ڲ���Ϣ
	float Pl0_0, Pl0_2, Pl1_1, Pl1_2;                //����ڲ���Ϣ
	float Pr0_0, Pr0_2, Pr0_3, Pr1_1, Pr1_2;                //����ڲ���Ϣ
	float Q0_3, Q1_3, Q2_3, Q3_2;                //����ڲ���Ϣ
	
	//�������ϵ������
	DoubleCameraCalibrationStruct() {
		cameraMatrixL0_0 = 874.692;
		cameraMatrixL0_2 = 528.006;
		cameraMatrixL1_1 = 873.565;
		cameraMatrixL1_2 = 296.077;
		distCoeffL0_0 = -0.0670002;
		distCoeffL1_0 = -0.0737268;
		distCoeffL2_0 = -0.00340482;
		distCoeffL3_0 = 0.0111696;
		distCoeffL4_0 = 2.2911;
		cameraMatrixR0_0 = 879.036;
		cameraMatrixR0_2 = 461.344;
		cameraMatrixR1_1 = 878.316;
		cameraMatrixR1_2 = 319.486;
		distCoeffR0_0 = -0.0675976;
		distCoeffR1_0 = -0.244388;
		distCoeffR2_0 = -0.00199048;
		distCoeffR3_0 = -0.00158195;
		distCoeffR4_0 = 1.58403;
		R0_0 = 0.998782;
		R0_1 = -0.00924025;
		R0_2 = 0.0484729;
		R1_0 = 0.00962545;
		R1_1 = 0.999924;
		R1_2 = -0.00771928;
		R2_0 = -0.0483979;
		R2_1 = 0.00817645;
		R2_2 = 0.998795;
		T0_0 = -70.9;
		T0_1 = 1.04618;
		T0_2 = -6.0273;
		Rl0_0 = 0.990844;
		Rl0_1 = -0.0232135;
		Rl0_2 = 0.133001;
		Rl1_0 = 0.0238848;
		Rl1_1 = 0.999709;
		Rl1_2 = -0.0034538;
		Rl2_0 = -0.132883;
		Rl2_1 = 0.00659889;
		Rl2_2 = 0.99111;
		Rr0_0 = 0.996298;
		Rr0_1 = -0.0147011;
		Rr0_2 = 0.0846965;
		Rr1_0 = 0.0144507;
		Rr1_1 = 0.999889;
		Rr1_2 = 0.00356845;
		Rr2_0 = -0.0847396;
		Rr2_1 = -0.00233132;
		Rr2_2 = 0.9964;
		Pl0_0 = 875.941;
		Pl0_2 = 368.489;
		Pl1_1 = 875.941;
		Pl1_2 = 306;
		Pr0_0 = 875.941;
		Pr0_2 = 368.489;
		Pr0_3 = -62335;
		Pr1_1 = 875.941;
		Pr1_2 = 306;
		Q0_3 = -368.489;
		Q1_3 = -306;
		Q2_3 = 875.941;
		Q3_2 = 0.0140522;
	}
};

class DCalibration {
public:
	DCalibration(std::string caliResultPath, std::string caliResultName, std::string leftImgPath, std::string rightImgPath, std::string leftCameraPath, std::string rightCameraPath, cv::Size boardSize) {
		this->caliResultPath = caliResultPath;
		this->caliResultName = caliResultName;
		this->leftImgPath = leftImgPath;
		this->rightImgPath = rightImgPath;
		this->leftCameraPath = leftCameraPath;
        this->rightCameraPath = rightCameraPath;
		this->boardSize = boardSize;
	}
	~DCalibration() {}


	/**
	 * ��ӡ�궨����
	 * @param calibrationData �궨�����ṹ��
	 */
	static void printCalibrationData(DoubleCameraCalibrationStruct calibrationData);

	/**
	 * ��������궨����
	 * @param filename �ļ���
	 * @return �궨�����ṹ��
	 */
	static DoubleCameraCalibrationStruct readCalibrationData(const std::string& filename);

	bool CaliResultInit(cv::Mat& cameraMatrixL, cv::Mat& distCoeffL, cv::Mat& cameraMatrixR, cv::Mat& distCoeffR);
	
	void CalRealPoint(std::vector<std::vector<cv::Point3f>>& obj, int boardwidth, int boardheight, int imgNumber, int squaresize);
	
	void CaliProcess();

	void Run();

	bool readPatternImg(const std::string& filepath, std::vector<cv::Mat>& patternImgList);

	static bool doubleCameraCali(const std::string& leftSerialNumber,const std::string& rightSerialNumber);
private:
	std::string caliResultPath;
	std::string caliResultName;
	std::string leftImgPath;
	std::string rightImgPath;
    std::string leftCameraPath;
    std::string rightCameraPath;

	int imgHeight;
	int imgWidth;
	int imgNum;
	const int  frameNumber = 20;    //����궨ʱ��Ҫ���óɹ���ͼ��֡��
	float scale = 0.5;
	cv::Size boardSize;
	cv::Mat R, T, E, F;
	std::vector<cv::Mat> patternLeftImgList;
	std::vector<cv::Mat> patternRightImgList;
	std::vector<cv::Mat> rvecs;
	std::vector<cv::Mat> tvecs;
	std::vector<std::vector<cv::Point2f>> imagePointL;      //��������������Ƭ�ǵ�����꼯��
	std::vector<std::vector<cv::Point2f>> imagePointR;      //�ұ������������Ƭ�ǵ�����꼯��
	std::vector<std::vector<cv::Point3f>> objRealPoint;     //����ͼ��Ľǵ��ʵ���������꼯��
	//����ͼ��Ľǵ��ʵ���������꼯�� 
	std::vector<cv::Point2f> cornerL;
	//��������ĳһ��Ƭ�ǵ����꼯�� 
	std::vector<cv::Point2f> cornerR;
	//�ұ������ĳһ��Ƭ�ǵ����꼯�� 
	cv::Mat rgbImageL, grayImageL;
	cv::Mat rgbImageR, grayImageR;
	cv::Mat Rl, Rr, Pl, Pr, Q;
	
	//У����ת����R��ͶӰ����P ��ͶӰ����Q  
	cv::Mat mapLx, mapLy, mapRx, mapRy;
	//ӳ���
	cv::Rect validROIL, validROIR;
	cv::Mat cameraMatrixL;          //������ڲ�
	cv::Mat distCoeffL;             //����ϵ��      /*���ȱ궨�õ���������ڲξ���fx 0 cx0 fy cy0 0  1*/
	cv::Mat distCoeffR;             //����ϵ��
	cv::Mat cameraMatrixR;          //������ڲ�

};


#endif