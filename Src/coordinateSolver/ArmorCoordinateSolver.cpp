#include "ArmorCoordinateSolver.h"

#define YAW_ANGLE 0
#define PITCH_ANGLE 0

ArmorCoordinateSolver::ArmorCoordinateSolver(DoubleCameraCalibrationStruct calibrationData) {
	//取数需要的矩阵参数
	_cameraMatrixL = (cv::Mat_<float>(3, 3) << calibrationData.cameraMatrixL0_0, 0, calibrationData.cameraMatrixL0_2,
		0, calibrationData.cameraMatrixL1_1, calibrationData.cameraMatrixL1_2,
		0, 0, 1);
	_cameraMatrixL.convertTo(_cameraMatrixL, CV_64F);
	_distCoeffL = (cv::Mat_<float>(5, 1) << calibrationData.distCoeffL0_0,
		calibrationData.distCoeffL1_0,
		calibrationData.distCoeffL2_0,
		calibrationData.distCoeffL3_0,
		calibrationData.distCoeffL4_0);
	_distCoeffL.convertTo(_distCoeffL, CV_64F);
	_cameraMatrixR = (cv::Mat_<float>(3, 3) << calibrationData.cameraMatrixR0_0, 0, calibrationData.cameraMatrixR0_2,
		0, calibrationData.cameraMatrixR1_1, calibrationData.cameraMatrixR1_2,
		0, 0, 1);
	_cameraMatrixR.convertTo(_cameraMatrixR, CV_64F);
	_distCoeffR = (cv::Mat_<float>(5, 1) << calibrationData.distCoeffR0_0,
		calibrationData.distCoeffR1_0,
		calibrationData.distCoeffR2_0,
		calibrationData.distCoeffR3_0,
		calibrationData.distCoeffR4_0);
	_distCoeffR.convertTo(_distCoeffR, CV_64F);
	_rotationMatrix = (cv::Mat_<float>(3, 3) << calibrationData.R0_0, calibrationData.R0_1, calibrationData.R0_2,
		calibrationData.R1_0, calibrationData.R1_1, calibrationData.R1_2,
		calibrationData.R2_0, calibrationData.R2_1, calibrationData.R2_2);
	_rotationMatrix.convertTo(_rotationMatrix, CV_64F);
	_translationMatrix = (cv::Mat_<float>(3, 1) << calibrationData.T0_0,
		calibrationData.T0_1,
		calibrationData.T0_2);
	_translationMatrix.convertTo(_translationMatrix, CV_64F);
	_rotationMatrixL = (cv::Mat_<float>(3, 3) << calibrationData.Rl0_0, calibrationData.Rl0_1, calibrationData.Rl0_2,
		calibrationData.Rl1_0, calibrationData.Rl1_1, calibrationData.Rl1_2,
		calibrationData.Rl2_0, calibrationData.Rl2_1, calibrationData.Rl2_2);
	_rotationMatrixL.convertTo(_rotationMatrixL, CV_64F);
	_rotationMatrixR = (cv::Mat_<float>(3, 3) << calibrationData.Rr0_0, calibrationData.Rr0_1, calibrationData.Rr0_2,
		calibrationData.Rr1_0, calibrationData.Rr1_1, calibrationData.Rr1_2,
		calibrationData.Rr2_0, calibrationData.Rr2_1, calibrationData.Rr2_2);
	_rotationMatrixR.convertTo(_rotationMatrixR, CV_64F);
	_projectionMatrixL = (cv::Mat_<float>(3, 4) << calibrationData.Pl0_0, 0, calibrationData.Pl0_2, 0,
		0, calibrationData.Pl1_1, calibrationData.Pl1_2, 0,
		0, 0, 1, 0);
	_projectionMatrixL.convertTo(_projectionMatrixL, CV_64F);
	_projectionMatrixR = (cv::Mat_<float>(3, 4) << calibrationData.Pr0_0, 0, calibrationData.Pr0_2, calibrationData.Pr0_3,
		0, calibrationData.Pr1_1, calibrationData.Pr1_2, 0,
		0, 0, 1, 0);
	_projectionMatrixR.convertTo(_projectionMatrixR, CV_64F);
	_depthMappingMatrix = (cv::Mat_<float>(4, 4) << 1, 0, 0, calibrationData.Q0_3,
		0, 1, 0, 0, calibrationData.Q1_3,
		0, 0, 0, calibrationData.Q2_3,
		0, 0, calibrationData.Q3_2, 0);
	_depthMappingMatrix.convertTo(_depthMappingMatrix, CV_64F);
	_c_x = calibrationData.Q0_3;
	_c_y = calibrationData.Q1_3;
	_f = calibrationData.Q2_3;

	 //基线
    _b = 1 / calibrationData.Q3_2;
	std::cout << "_c_x:" << _c_x << "	" << "_c_y:" << _c_y << " baseline:" << _b << "	" << "f:" << _f << std::endl;

}

void ArmorCoordinateSolver::GetResult(const cv::Rect leftBoxes, const cv::Rect rightBoxes)
{
	_leftCameraCenter.x =leftBoxes.x + leftBoxes.width/2;
	_rightCameraCenter.x =rightBoxes.x + rightBoxes.width / 2;
	_leftCameraCenter.y =leftBoxes.y + leftBoxes.height / 2;
}


void ArmorCoordinateSolver::SteroTo3D()
{
	//三角测量  视差和基线的关系
    _actualCoordinate.x = (_leftCameraCenter.x + _c_x) * _b / (_leftCameraCenter.x - _rightCameraCenter.x);
    _actualCoordinate.y = (_leftCameraCenter.y + _c_y) * _b / (_leftCameraCenter.x - _rightCameraCenter.x);
    _actualCoordinate.z = fabs(_f * _b / (_leftCameraCenter.x - _rightCameraCenter.x));
}

void ArmorCoordinateSolver::actualCoordinateSolver() {
    //yaw轴方向转化
    _actualCoordinate.x = _actualCoordinate.x * cos(YAW_ANGLE);
    _actualCoordinate.z = _actualCoordinate.z * cos(YAW_ANGLE);

    //pitch轴方向转化
    _actualCoordinate.y = _actualCoordinate.y * cos(PITCH_ANGLE);
    _actualCoordinate.z = _actualCoordinate.z * cos(PITCH_ANGLE);

	_areCoordinate = _actualCoordinate + _cameraCoordinate;
}

cv::Point3f ArmorCoordinateSolver::ArmorProcess(const cv::Rect leftBoxes, const cv::Rect rightBoxes)
{
	GetResult(leftBoxes, rightBoxes);
	SteroTo3D();
	actualCoordinateSolver();
	return _areCoordinate;
}
