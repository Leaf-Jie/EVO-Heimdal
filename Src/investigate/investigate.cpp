#include "investigate.h"

//���캯������Ա������ʼ��
Investigate::Investigate()
{
	//����ģ�ͼ���
	readModel();

	string modelConfig = "../TrainData/yolo/yolov4-radar.cfg";
	string modelWeights = "../TrainData/yolo/yolov4-radar_last.weights";
	this->detector = new Detector(modelConfig, modelWeights, 0);
}


void Investigate::readModel() {
	string classesFile = "../TrainData/yolo/radar.names";
	//���������
	ifstream ifs(classesFile.c_str());
	string line;
	while (getline(ifs, line)) classes.push_back(line);

}

//��ȡ�������
bool Investigate::extractingInformation(cv::Mat& frame, int classId, float conf, int left, int top, int right, int bottom)
{
	//������Ҫ�ĳ�����Ϣ
    if(enemyColor_ == ENEMY_RED) {
        if (classId == CAR_RED || classId == WATCHER)
            return informationScreening(frame, classId, conf, left, top, right, bottom);
    } else{
        if (classId == CAR_BLUE || classId == WATCHER)
            return informationScreening(frame, classId, conf, left, top, right, bottom);
	}
	return false;
}

bool Investigate::informationScreening(cv::Mat &frame, int classId, float conf, int left, int top, int right,int bottom) {
    if (conf > 0.8) {
        if ((right - left) > 1200 || (bottom - top) > 600) {
            return false;
        }
        if (classId == WATCHER) {
            sentryId = reasonableOuts.size();
            sentryFound = true;
        }
        cv::Rect box = cv::Rect(cv::Point(left, top), cv::Point(right, bottom));
        return true;
    }
}

void Investigate::backgroundSubtraction(bbox_t box, cv::Mat back, cv::Rect carROI) {
    Util::makeRectSafe(carROI, back.size());
    Util::makeRectSafe(carROI, frame.size());
    cv::Mat backSrc = back(carROI);
    cv::Mat carSrc = frame(carROI);
    cv::cvtColor(carSrc, carSrc, cv::COLOR_BGR2GRAY);
    cv::absdiff(carSrc, backSrc, carSrc);
    cv::threshold(carSrc, carSrc, 50, 255, cv::THRESH_BINARY);
    cv::dilate(carSrc, carSrc, Util::structuringElement7());
    for (int k = 0; k < carSrc.rows; ++k) {
        for (int j = 0; j < carSrc.cols; ++j) {
            if (carSrc.at<uchar>(k, j))
                oneWeight++;
        }
        oneWeight = oneWeight / (double)(box.w * box.h);
        box.prob += oneWeight;
        oneWeight = 0.0;
    }
}

void Investigate::ScreeningBox( vector<bbox_t> outs, bool cameraPosition) {
    reasonableOuts.clear();
    for (int i = 0; i < outs.size(); i++) {
            cv::Rect carROI = cv::Rect(outs[i].x, outs[i].y, outs[i].w, outs[i].h);
            if(cameraPosition == CAMERA_LEFT){
                backgroundSubtraction(outs[i], leftBack, carROI);
            }
            else
            {
                backgroundSubtraction(outs[i], rightBack, carROI);
            }
            //��ȡ��Ҫ�ĳ���
            if(extractingInformation(frame, outs[i].obj_id, outs[i].prob, outs[i].x, outs[i].y,
                              outs[i].x + outs[i].w, outs[i].y + outs[i].h))
            {
                reasonableOuts.push_back(outs[i]);
            }
    }
}

//ʹ��׷����
void Investigate::trackCar(cv::Mat& frame, vector<bbox_t>& outs)
{
    if (oldOuts.size() != outs.size()) {
        trackerFlow.update_tracking_flow(frame, outs);
        while (trackOptFlow.size() > 0) {
            outs = trackerFlow.tracking_flow(trackOptFlow.front(), false);
            trackOptFlow.pop();
            oldOuts = outs;
            outs = trackKalman.correct(outs);
        }
    }
    else{
        trackOptFlow.push(frame);
        outs = trackerFlow.tracking_flow(frame, false);
        outs = trackKalman.predict();
    }
}

void Investigate::process(const cv::Mat& src, std::vector<bbox_t>& correctOuts, bool cameraPosition, EnemyColor enemyColor) {
    enemyColor_ = enemyColor;
    frame = src;
    //ȡ��һ֡ͼ��
    if (!firsLeftMat && cameraPosition == CAMERA_LEFT) {;
        leftBack = frame.clone();
        cv::cvtColor(leftBack, leftBack, cv::COLOR_BGR2GRAY);
        firsLeftMat = true;
    }
    if (!firsRightMat && cameraPosition == CAMERA_RIGHT) {
        rightBack = frame.clone();
        cv::cvtColor(rightBack, rightBack, cv::COLOR_BGR2GRAY);
        firsRightMat = true;
    }

    //Matͼ��תΪyolo�����ʽ
    shared_ptr<image_t> detImg = detector->mat_to_image_resize(frame);
    //ǰ��Ԥ��
    vector<bbox_t> outs = detector->detect_resized(*detImg, frame.cols, frame.rows, 0.2);
    //׷��
    trackCar(frame, outs);
    //�������
    ScreeningBox(outs, cameraPosition);
    //������Ч��Ϣ
    correctOuts = reasonableOuts;

}