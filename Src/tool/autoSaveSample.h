#ifndef _AUTO_SAVE_SAMPLE_H_
#define _AUTO_SAVE_SAMPLE_H_

#include "Infra/Thread.h"
#include "decisionLevel/decisionLevel.h"
#include "string"

using namespace std;

#define SAVE_INTERVAL_TIME  1.0f


class AutoSaveSample {
public:
    AutoSaveSample(DecisionLevelPtr &DecisionLevelPtr, SaveSampleMode mode = SAVE_DISABLE);

    /**
     * 保存样本外部调用
     */
    void saveSample();

private:

    double saveTime_;

    SaveSampleMode _saveSampleMode = SAVE_DISABLE;

    DecisionLevelPtr decisionLevelPtr;

    uint32_t _savePositiveNumber = 0;
    uint32_t _saveNegativeNumber = 0;

    uint32_t _saveBuffPositiveNumber = 0;
    uint32_t _saveBuffNegativeNumber = 0;

    uint32_t _saveCirclePositiveNumber = 0;
    uint32_t _saveCircleNegativeNumber = 0;

    string _autoSaveTrueSamplePath = AUTO_SAVE_TRUE_SAMPLE_PATH;
    string _autoSaveFalseSamplePath = AUTO_SAVE_FALSE_SAMPLE_PATH;

    /**
     * 获取已有的样本容量
     */
    void getImgSampleSize();

    /**
     * 从决策线程获取是否准备好保存样本，限制样本保存时间间隔为 SAVE_INTERVAL_TIME 秒以上
     * @return flag
     */
    bool saveSamplesReady();

    /**
     * 根据识别模式和样本标签自动保存样本数据
     * @param sampleData 样本数据
     */
    void saveSampleByMode(const struct SampleDataStruct &sampleData);
};

#endif
