#include"doubleCameraCalibration.h"

DoubleCameraCalibrationStruct DCalibration::readCalibrationData(const std::string& filename)
{
	LOG::info(" camera calibration data in " + filename);
    std::cout <<  filename << std::endl;
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        LOG::error("Can not read camera calibration data in " + filename + ",use default param!");
        return DoubleCameraCalibrationStruct();
    } else {
        LOG::info("Read camera calibration data success in " + filename);
        try {
            DoubleCameraCalibrationStruct calibrationStruct;
            fs["cameraMatrixL0_0"] >> calibrationStruct.cameraMatrixL0_0;
            fs["cameraMatrixL0_2"] >> calibrationStruct.cameraMatrixL0_2;
            fs["cameraMatrixL1_1"] >> calibrationStruct.cameraMatrixL1_1;
            fs["cameraMatrixL1_2"] >> calibrationStruct.cameraMatrixL1_2;
            fs["distCoeffL0_0"] >> calibrationStruct.distCoeffL0_0;
            fs["distCoeffL1_0"] >> calibrationStruct.distCoeffL1_0;
            fs["distCoeffL2_0"] >> calibrationStruct.distCoeffL2_0;
            fs["distCoeffL3_0"] >> calibrationStruct.distCoeffL3_0;
            fs["distCoeffL4_0"] >> calibrationStruct.distCoeffL4_0;
            fs["cameraMatrixR0_0"] >> calibrationStruct.cameraMatrixR0_0;
            fs["cameraMatrixR0_2"] >> calibrationStruct.cameraMatrixR0_2;
            fs["cameraMatrixR1_1"] >> calibrationStruct.cameraMatrixR1_1;
            fs["cameraMatrixR1_2"] >> calibrationStruct.cameraMatrixR1_2;
            fs["distCoeffR0_0"] >> calibrationStruct.distCoeffR0_0;
            fs["distCoeffR1_0"] >> calibrationStruct.distCoeffR1_0;
            fs["distCoeffR2_0"] >> calibrationStruct.distCoeffR2_0;
            fs["distCoeffR3_0"] >> calibrationStruct.distCoeffR3_0;
            fs["distCoeffR4_0"] >> calibrationStruct.distCoeffR4_0;
            fs["R0_0"] >> calibrationStruct.R0_0;
            fs["R0_1"] >> calibrationStruct.R0_1;
            fs["R0_2"] >> calibrationStruct.R0_2;
            fs["R1_0"] >> calibrationStruct.R1_0;
            fs["R1_1"] >> calibrationStruct.R1_1;
            fs["R1_2"] >> calibrationStruct.R1_2;
            fs["R2_0"] >> calibrationStruct.R2_0;
            fs["R2_1"] >> calibrationStruct.R2_1;
            fs["R2_2"] >> calibrationStruct.R2_2;
            fs["T0_0"] >> calibrationStruct.T0_0;
            fs["T0_1"] >> calibrationStruct.T0_1;
            fs["T0_2"] >> calibrationStruct.T0_2;
            fs["Rl0_0"] >> calibrationStruct.Rl0_0;
            fs["Rl0_1"] >> calibrationStruct.Rl0_1;
            fs["Rl0_2"] >> calibrationStruct.Rl0_2;
            fs["Rl1_0"] >> calibrationStruct.Rl1_0;
            fs["Rl1_1"] >> calibrationStruct.Rl1_1;
            fs["Rl1_2"] >> calibrationStruct.Rl1_2;
            fs["Rl2_0"] >> calibrationStruct.Rl2_0;
            fs["Rl2_1"] >> calibrationStruct.Rl2_1;
            fs["Rl2_2"] >> calibrationStruct.Rl2_2;
            fs["Rr0_0"] >> calibrationStruct.Rr0_0;
            fs["Rr0_1"] >> calibrationStruct.Rr0_1;
            fs["Rr0_2"] >> calibrationStruct.Rr0_2;
            fs["Rr1_0"] >> calibrationStruct.Rr1_0;
            fs["Rr1_1"] >> calibrationStruct.Rr1_1;
            fs["Rr1_2"] >> calibrationStruct.Rr1_2;
            fs["Rr2_0"] >> calibrationStruct.Rr2_0;
            fs["Rr2_1"] >> calibrationStruct.Rr2_1;
            fs["Rr2_2"] >> calibrationStruct.Rr2_2;
            fs["Pl0_0"] >> calibrationStruct.Pl0_0;
            fs["Pl0_2"] >> calibrationStruct.Pl0_2;
            fs["Pl1_1"] >> calibrationStruct.Pl1_1;
            fs["Pl1_2"] >> calibrationStruct.Pl1_2;
            fs["Pr0_0"] >> calibrationStruct.Pr0_0;
            fs["Pr0_2"] >> calibrationStruct.Pr0_2;
            fs["Pr0_3"] >> calibrationStruct.Pr0_3;
            fs["Pr1_1"] >> calibrationStruct.Pr1_1;
            fs["Pr1_2"] >> calibrationStruct.Pr1_2;
            fs["Q0_3"] >> calibrationStruct.Q0_3;
            fs["Q1_3"] >> calibrationStruct.Q1_3;
            fs["Q2_3"] >> calibrationStruct.Q2_3;
            fs["Q3_2"] >> calibrationStruct.Q3_2;
            fs.release();
            return calibrationStruct;
        } catch (cv::Exception &e) {
            fs.release();
            LOG::error(e.msg.substr(0, e.msg.length() - 1));
            LOG::error("catch exception while reading calibration data! Use default param!");
            return DoubleCameraCalibrationStruct();
        }
    }
}
bool DCalibration::CaliResultInit(cv::Mat& cameraMatrixL, cv::Mat& distCoeffL, cv::Mat& cameraMatrixR, cv::Mat& distCoeffR)
{
	float fxL, fyL, uL, vL, k1L, k2L, p1L, p2L, k3L;
	float fxR, fyR, uR, vR, k1R, k2R, p1R, p2R, k3R;
	cv::FileStorage fsL(leftCameraPath, cv::FileStorage::READ);
	if (!fsL.isOpened()) {
		LOG::error("Can not read camera calibration data in " + leftCameraPath + ",use default param!");
        return false;
	}
	else {
		LOG::info("Read leftCamera calibration data success in " + leftCameraPath);
		fsL["IntrinsicParameters_fx"] >> fxL;
		fsL["IntrinsicParameters_fy"] >> fyL;
		fsL["IntrinsicParameters_u0"] >> uL;
		fsL["IntrinsicParameters_v0"] >> vL;
		fsL["DistortionFactor_k1"] >> k1L;
		fsL["DistortionFactor_k2"] >> k2L;
        fsL["DistortionFactor_p1"] >> p1L;
        fsL["DistortionFactor_p2"] >> p2L;
		fsL["DistortionFactor_k3"] >> k3L;
		fsL.release();
	}
	cameraMatrixL = (cv::Mat_<double>(3, 3) << fxL, 0, fyL, 0, uL, vL, 0, 0, 1);
	distCoeffL = (cv::Mat_<double>(5, 1) << k1L, k2L, p1L, p2L, k3L);         //畸变系数

	cv::FileStorage fsR(rightCameraPath, cv::FileStorage::READ);
	if (!fsR.isOpened()) {
		LOG::error("Can not read camera calibration data in " + rightCameraPath + ",use default param!");
        return false;
	}
	else {
		LOG::info("Read leftCamera calibration data success in " + rightCameraPath);
		fsR["IntrinsicParameters_fx"] >> fxR;
		fsR["IntrinsicParameters_fy"] >> fyR;
		fsR["IntrinsicParameters_u0"] >> uR;
		fsR["IntrinsicParameters_v0"] >> vR;
		fsR["DistortionFactor_k1"] >> k1R;
		fsR["DistortionFactor_k2"] >> k2R;
		fsR["DistortionFactor_p1"] >> p1R;
        fsR["DistortionFactor_p2"] >> p2R;
        fsR["DistortionFactor_k3"] >> k3R;
		fsR.release();
	}
	cameraMatrixR = (cv::Mat_<double>(3, 3) << fxR, 0, fyR, 0, uR, vR, 0, 0, 1);
	distCoeffR = (cv::Mat_<double>(5, 1) << k1R, k2R, p1R, p2R, k3R);         //畸变系数;
	return true;
}

bool DCalibration::readPatternImg(const std::string& filepath, std::vector<cv::Mat>& patternImgList)
{
	uint32_t imgCount = 0;
	FileOperation::getImgFileInOrder(filepath, imgCount, "jpg", patternImgList, 1);
	if (imgCount == 0) {
        LOG::error("Can not find !" + filepath + "image");
	    return false;
	}
	this->imgNum = imgCount;
	imgHeight = patternImgList[0].rows;
	imgWidth = patternImgList[0].cols;
	return true;
}


void DCalibration::CalRealPoint(std::vector<std::vector<cv::Point3f>>& obj, int boardwidth, int boardheight, int imgNumber, int squaresize)
{
	std::vector<cv::Point3f> imgpoint;
	for (int rowIndex = 0; rowIndex < boardheight; rowIndex++) {
		for (int colIndex = 0; colIndex < boardwidth; colIndex++)
		{
			imgpoint.push_back(cv::Point3f(rowIndex * squaresize, colIndex * squaresize, 0));
		}
	}
	for (int imgIndex = 0; imgIndex < imgNumber; imgIndex++)
	{
		obj.push_back(imgpoint);
	}
}

void DCalibration::CaliProcess()
{
	cv::Mat imageL, imageR;
	int goodFrameCount = 0;
	int pictureNumber = 0;
	const int squareSize = 30;      //标定板黑白格子的大小 单位mm
	cv::Size imgSize = cv::Size(imgWidth, imgHeight);//摄像头的分辨率
    LOG::info("********Start extracting the corners! ********");
	while (goodFrameCount < frameNumber) 
	{
        LOG::info("Image#" + std::to_string(pictureNumber) + ".......");
		imageL = patternLeftImgList[pictureNumber].clone();
		imageR = patternRightImgList[pictureNumber].clone();
        /*对左边的图像下采样*/
		cv::resize(imageL, rgbImageL, cv::Size(), scale, scale, cv::INTER_LINEAR);
		cvtColor(rgbImageL, grayImageL, cv::COLOR_BGR2GRAY);									//转化灰度图
		/*对右边的图像下采样*/
		cv::resize(imageR, rgbImageR, cv::Size(), scale, scale, cv::INTER_LINEAR);
        cvtColor(rgbImageR, grayImageR, cv::COLOR_BGR2GRAY);

		bool isFindL, isFindR;
		isFindL = findChessboardCorners(grayImageL, boardSize, cornerL, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK);				//寻找角点	返回值判断是否找到角点
		isFindR = findChessboardCorners(grayImageR, boardSize, cornerR, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK);				//寻找角点  返回值判断是否找到角点
        pictureNumber++;
		if (isFindL == true && isFindR == true)  //如果两幅图像都找到了所有的角点 则说明这两幅图像是可行的  
		{	
			//上采样corner
			for (size_t num = 0; num < cornerL.size(); num++) {
				cv::Point2f tempPointL = cornerL[num];
				cornerL[num] = cv::Point2f(tempPointL.x / scale, tempPointL.y / scale);
				cv::Point2f tempPointR = cornerR[num];
				cornerR[num] = cv::Point2f(tempPointR.x / scale, tempPointR.y / scale);
			}

            //画出角点
			cornerSubPix(grayImageL, cornerL, cv::Size(5, 5), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 20, 0.1));// CV_TERMCRIT_EPS							//亚像素角点
			drawChessboardCorners(imageL, boardSize, cornerL, isFindL);
			imshow("chessboardL", imageL);
            cornerSubPix(grayImageR, cornerR, cv::Size(5, 5), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 20, 0.1));
            drawChessboardCorners(imageR, boardSize, cornerR, isFindR);
            imshow("chessboardR",imageR);
            //储存找到的角点
			imagePointL.push_back(cornerL);
			imagePointR.push_back(cornerR);
			goodFrameCount++;
		}
		else
		{
            LOG::error("Can not find chess board corners!");
            continue;
		}
		cv::waitKey(1) ;
	}	/*	计算实际的校正点的三维坐标	根据实际标定格子的大小来设置	*/
	CalRealPoint(objRealPoint, boardSize.width, boardSize.height, frameNumber, squareSize);
	std::cout << "cal real successful" << std::endl;
	/*	标定摄像头	由于左右摄像机分别都经过了单目标定	所以在此处选择flag = CALIB_USE_INTRINSIC_GUESS	*/
	double rms = stereoCalibrate(objRealPoint,
		imagePointL, imagePointR,
		cameraMatrixL, distCoeffL,
		cameraMatrixR, distCoeffR,
		imgSize, R, T, E, F, cv::CALIB_USE_INTRINSIC_GUESS,
		cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 50, 1e-5));				//对摄像头进行标定

	std::cout << "Stereo Calibration done with RMS error = " << rms << std::endl;
	stereoRectify(cameraMatrixL, distCoeffL, cameraMatrixR, distCoeffR, imgSize, R, T, Rl, Rr, Pl, Pr, Q,
		cv::CALIB_ZERO_DISPARITY, -1, imgSize, &validROIL, &validROIR);
}

void DCalibration::Run()
{
    bool readLeftPatternImgSuccessful = readPatternImg(leftImgPath, patternLeftImgList);
    bool readRightPatternImgSuccessful = readPatternImg(rightImgPath, patternRightImgList);
	bool readCaliFile = CaliResultInit(cameraMatrixL, distCoeffL, cameraMatrixR, distCoeffR);
	if (!readCaliFile && readLeftPatternImgSuccessful && readRightPatternImgSuccessful) {
		LOG::error("Please check file path!");
        exit(0);
	}
	CaliProcess();
	exit(0);
}

void DCalibration::printCalibrationData(DoubleCameraCalibrationStruct calibrationData) {
	LOG::debug("DoubleCameraCaliData.cameraMatrixL0_0= " + std::to_string(calibrationData.cameraMatrixL0_0));
	LOG::debug("DoubleCameraCaliData.cameraMatrixL0_2= " + std::to_string(calibrationData.cameraMatrixL0_2));
	LOG::debug("DoubleCameraCaliData.cameraMatrixL1_1= " + std::to_string(calibrationData.cameraMatrixL1_1));
	LOG::debug("DoubleCameraCaliData.cameraMatrixL1_2= " + std::to_string(calibrationData.cameraMatrixL1_2));
	LOG::debug("DoubleCameraCaliData.distCoeffL0_0= " + std::to_string(calibrationData.distCoeffL0_0));
	LOG::debug("DoubleCameraCaliData.distCoeffL1_0= " + std::to_string(calibrationData.distCoeffL1_0));
	LOG::debug("DoubleCameraCaliData.distCoeffL2_0= " + std::to_string(calibrationData.distCoeffL2_0));
	LOG::debug("DoubleCameraCaliData.distCoeffL3_0= " + std::to_string(calibrationData.distCoeffL3_0));
	LOG::debug("DoubleCameraCaliData.distCoeffL4_0= " + std::to_string(calibrationData.distCoeffL4_0));
	LOG::debug("DoubleCameraCaliData.cameraMatrixR0_0= " + std::to_string(calibrationData.cameraMatrixR0_0));
	LOG::debug("DoubleCameraCaliData.cameraMatrixR0_2= " + std::to_string(calibrationData.cameraMatrixR0_2));
	LOG::debug("DoubleCameraCaliData.cameraMatrixR1_1= " + std::to_string(calibrationData.cameraMatrixR1_1));
	LOG::debug("DoubleCameraCaliData.cameraMatrixR1_2= " + std::to_string(calibrationData.cameraMatrixR1_2));
	LOG::debug("DoubleCameraCaliData.distCoeffR0_0= " + std::to_string(calibrationData.distCoeffR0_0));
	LOG::debug("DoubleCameraCaliData.distCoeffR1_0= " + std::to_string(calibrationData.distCoeffR1_0));
	LOG::debug("DoubleCameraCaliData.distCoeffR2_0= " + std::to_string(calibrationData.distCoeffR2_0));
	LOG::debug("DoubleCameraCaliData.distCoeffR3_0= " + std::to_string(calibrationData.distCoeffR3_0));
	LOG::debug("DoubleCameraCaliData.distCoeffR_0= " + std::to_string(calibrationData.distCoeffR4_0));
	LOG::debug("DoubleCameraCaliData.R0_0= " + std::to_string(calibrationData.R0_0));
	LOG::debug("DoubleCameraCaliData.R0_1= " + std::to_string(calibrationData.R0_1));
	LOG::debug("DoubleCameraCaliData.R0_2= " + std::to_string(calibrationData.R0_2));
	LOG::debug("DoubleCameraCaliData.R1_0= " + std::to_string(calibrationData.R1_0));
	LOG::debug("DoubleCameraCaliData.R1_1= " + std::to_string(calibrationData.R1_1));
	LOG::debug("DoubleCameraCaliData.R1_2= " + std::to_string(calibrationData.R1_2));
	LOG::debug("DoubleCameraCaliData.R2_0= " + std::to_string(calibrationData.R2_0));
	LOG::debug("DoubleCameraCaliData.R2_1= " + std::to_string(calibrationData.R2_1));
	LOG::debug("DoubleCameraCaliData.R2_2= " + std::to_string(calibrationData.R2_2));
	LOG::debug("DoubleCameraCaliData.T0_0= " + std::to_string(calibrationData.T0_0));
	LOG::debug("DoubleCameraCaliData.T0_1= " + std::to_string(calibrationData.T0_1));
	LOG::debug("DoubleCameraCaliData.T0_2= " + std::to_string(calibrationData.T0_2));
	LOG::debug("DoubleCameraCaliData.Rl0_0= " + std::to_string(calibrationData.Rl0_0));
	LOG::debug("DoubleCameraCaliData.Rl0_1= " + std::to_string(calibrationData.Rl0_1));
	LOG::debug("DoubleCameraCaliData.Rl0_2= " + std::to_string(calibrationData.Rl0_2));
	LOG::debug("DoubleCameraCaliData.Rl1_0= " + std::to_string(calibrationData.Rl1_0));
	LOG::debug("DoubleCameraCaliData.Rl1_1= " + std::to_string(calibrationData.Rl1_1));
	LOG::debug("DoubleCameraCaliData.Rl1_2= " + std::to_string(calibrationData.Rl1_2));
	LOG::debug("DoubleCameraCaliData.Rl2_0= " + std::to_string(calibrationData.Rl2_0));
	LOG::debug("DoubleCameraCaliData.Rl2_1= " + std::to_string(calibrationData.Rl2_1));
	LOG::debug("DoubleCameraCaliData.Rl2_2= " + std::to_string(calibrationData.Rl2_2));
	LOG::debug("DoubleCameraCaliData.Rr0_0= " + std::to_string(calibrationData.Rr0_0));
	LOG::debug("DoubleCameraCaliData.Rr0_1= " + std::to_string(calibrationData.Rr0_1));
	LOG::debug("DoubleCameraCaliData.Rr0_2= " + std::to_string(calibrationData.Rr0_2));
	LOG::debug("DoubleCameraCaliData.Rr1_0= " + std::to_string(calibrationData.Rr1_0));
	LOG::debug("DoubleCameraCaliData.Rr1_1= " + std::to_string(calibrationData.Rr1_1));
	LOG::debug("DoubleCameraCaliData.Rr1_2= " + std::to_string(calibrationData.Rr1_2));
	LOG::debug("DoubleCameraCaliData.Rr2_0= " + std::to_string(calibrationData.Rr2_0));
	LOG::debug("DoubleCameraCaliData.Rr2_1= " + std::to_string(calibrationData.Rr2_1));
	LOG::debug("DoubleCameraCaliData.Rr2_2= " + std::to_string(calibrationData.Rr2_2));
	LOG::debug("DoubleCameraCaliData.Pl0_0= " + std::to_string(calibrationData.Pl0_0));
	LOG::debug("DoubleCameraCaliData.Pl0_2= " + std::to_string(calibrationData.Pl0_2));
	LOG::debug("DoubleCameraCaliData.Pl1_1= " + std::to_string(calibrationData.Pl1_1));
	LOG::debug("DoubleCameraCaliData.Pl1_2= " + std::to_string(calibrationData.Pl1_2));
	LOG::debug("DoubleCameraCaliData.Pr0_0= " + std::to_string(calibrationData.Pr0_0));
	LOG::debug("DoubleCameraCaliData.Pr0_2= " + std::to_string(calibrationData.Pr0_2));
	LOG::debug("DoubleCameraCaliData.Pr0_3= " + std::to_string(calibrationData.Pr0_3));
	LOG::debug("DoubleCameraCaliData.Pr1_1= " + std::to_string(calibrationData.Pr1_1));
	LOG::debug("DoubleCameraCaliData.Pr1_2= " + std::to_string(calibrationData.Pr1_2));
	LOG::debug("DoubleCameraCaliData.Q0_3= " + std::to_string(calibrationData.Q0_3));
	LOG::debug("DoubleCameraCaliData.Q1_3= " + std::to_string(calibrationData.Q1_3));
	LOG::debug("DoubleCameraCaliData.Q2_3= " + std::to_string(calibrationData.Q2_3));
	LOG::debug("DoubleCameraCaliData.Q3_2= " + std::to_string(calibrationData.Q3_2));

}

//用于摄像头标定的外部调用接口
bool DCalibration::doubleCameraCali(const std::string& leftSerialNumber,const std::string& rightSerialNumber)
{
	//标定图存储的地址
	std::string leftPatternImgPath = LEFT_PATTERN_IMG_PATH;
	FileOperation::createDirectory(leftPatternImgPath);
	std::string rightPatternImgPath = RIGHT_PATTERN_IMG_PATH;
	FileOperation::createDirectory(rightPatternImgPath);
	//标定相机参数文件存储的地址
	std::string caliResultPath = CALI_RESULTS_PATH;
	FileOperation::createDirectory(caliResultPath);
	//标定相机参数文件名
	std::string caliCameraDataName = DOUBLE_CALI_CAMERA_DATA_NAME;
    //标定左相机参数文件
    std::string caliLeftCameraData = CALI_RESULTS_PATH + leftSerialNumber + ".yml";
    //标定右相机参数文件
    std::string caliRightCameraData = CALI_RESULTS_PATH + rightSerialNumber + ".yml";
	//标定棋盘图像的行列数
	cv::Size boardSize = cv::Size(11, 8);
	//双目标定初始化
	DCalibration calibration(caliResultPath, caliCameraDataName, leftPatternImgPath, rightPatternImgPath, caliLeftCameraData, caliRightCameraData, boardSize);
	//开始双目标定
	calibration.Run();
	return true;
}
