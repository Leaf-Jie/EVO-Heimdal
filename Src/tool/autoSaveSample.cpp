#include "autoSaveSample.h"

#define IMG_FORMAT ".bmp"

using namespace std;


AutoSaveSample::AutoSaveSample(DecisionLevelPtr& DecisionLevelPtr, SaveSampleMode mode)
	: decisionLevelPtr(DecisionLevelPtr) {
	_saveSampleMode = mode;
	//创建保存目录
	FileOperation::createDirectory(AUTO_SAVE_TRUE_SAMPLE_PATH);
	FileOperation::createDirectory(AUTO_SAVE_FALSE_SAMPLE_PATH);


	//获得现有的样本容量
	getImgSampleSize();
	saveTime_ = (double)cv::getTickCount();
}

bool AutoSaveSample::saveSamplesReady() {
	double time = ((double)cv::getTickCount() - saveTime_) / cv::getTickFrequency();
	return decisionLevelPtr->getCaptureState() && time > SAVE_INTERVAL_TIME;
}

void AutoSaveSample::getImgSampleSize() {

	_savePositiveNumber = FileOperation::getFileSizeInOrder(_autoSaveTrueSamplePath, IMG_FORMAT);
	_saveNegativeNumber = FileOperation::getFileSizeInOrder(_autoSaveFalseSamplePath, IMG_FORMAT);

}

void AutoSaveSample::saveSampleByMode(const struct SampleDataStruct& sampleData) {

		if (sampleData.classifyState) {
			FileOperation::saveImg(_autoSaveTrueSamplePath, _savePositiveNumber, sampleData.image, IMG_FORMAT);
		}
		else {
			FileOperation::saveImg(_autoSaveFalseSamplePath, _saveNegativeNumber, sampleData.image, IMG_FORMAT);
		}

}

void AutoSaveSample::saveSample() {
	switch (_saveSampleMode) {
	case SAVE_DISABLE: {
	}
					 break;
	case AUTO_SAVE: {
		if (saveSamplesReady()) {
			saveSampleByMode(decisionLevelPtr->getSampleData());
			saveTime_ = (double)cv::getTickCount();
		}
	}
	default:
		break;
	}
}

