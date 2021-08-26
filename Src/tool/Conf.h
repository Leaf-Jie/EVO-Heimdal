#ifndef _CONF_H_
#define _CONF_H_

#include "tool/filePath.h"

//相机参数
struct CameraConfiguration {
    double gainValue = 1.5;                       //增益
    int exposureValue = 5000;                   //曝光时间
    double gammaValue = 0.6;                      //gamma
};

class CameraConfigurationFactory {
private:
    static CameraConfigurationFactory& instance() {
        static CameraConfigurationFactory cameraConfigurationFactory;
        return cameraConfigurationFactory;
    }

    /**
     * 向配置文件中写入一个相机配置节点
     * @param cameraConfiguration 待写入的节点
     * @param name 节点名称
     * @param comment 待写入的注释
     * @param fs 配置文件指针
     */
    static void writeCameraConfigurationStruct(CameraConfiguration& cameraConfiguration, const char* name, const char* comment, cv::FileStorage& fs) {
        fs.writeComment("\n");
        fs.writeComment(comment);
        fs << name << "{"
            << "gainValue" << cameraConfiguration.gainValue
            << "exposureValue" << cameraConfiguration.exposureValue
            << "gammaValue" << cameraConfiguration.gammaValue
            << "}";
    }

    /**
     * 从配置文件中读入一个相机配置节点
     * @param cameraConfiguration 待读入的节点
     * @param name 节点名称
     * @param fs 配置文件指针
     */
    static void readCameraConfigurationStruct(CameraConfiguration& cameraConfiguration, const char* name, cv::FileStorage& fs) {
        cv::FileNode node = fs[name];
        node["gammaValue"] >> cameraConfiguration.gammaValue;
        node["exposureValue"] >> cameraConfiguration.exposureValue;
        node["gainValue"] >> cameraConfiguration.gainValue;
        LOG::debug(LOG::tostring(name) + ".gammaValue=" + std::to_string(cameraConfiguration.gammaValue));
        LOG::debug(LOG::tostring(name) + ".exposureValue=" + std::to_string(cameraConfiguration.exposureValue));
        LOG::debug(LOG::tostring(name) + ".gainValue=" + std::to_string(cameraConfiguration.gainValue));
    }

public:
    static CameraConfiguration getArmorCameraConfiguration() {
        return instance().cameraConfiguration;
    }


    static void resetAllConfig() {
        instance().cameraConfiguration = CameraConfiguration();
    }

    static void readCameraConfiguration(const std::string& filename) {
        cv::FileStorage fs(filename, cv::FileStorage::READ);
        if (!fs.isOpened()) {
            writeCameraConfiguration(filename);
            LOG::error("default config not find, write config to " + filename);
        }
        else {
            readCameraConfigurationStruct(instance().cameraConfiguration, "CameraConfiguration", fs);
        }
        fs.release();
    }

    static void writeCameraConfiguration(const std::string& filename) {
        cv::FileStorage fs(filename, cv::FileStorage::WRITE);
        //写入注释
        fs.writeComment("相机设置");
        writeCameraConfigurationStruct(instance().cameraConfiguration, "CameraConfiguration", "相机参数", fs);
        //销毁
        fs.release();
    }

public:

    CameraConfiguration cameraConfiguration;

};

class CodeSet {
private:
    bool useDebugGuiFlag = false;
    bool takePictureFlag = false;
    int grayAverage = 20;
    bool useAutoSaveSampleMode = 0;  //0:停止保存 1:自动保存    默认:0

    static CodeSet& instance() {
        static CodeSet codeSet;
        return codeSet;
    }

public:
    static void resetAllConfig() {
        instance() = CodeSet();
    }

    static void readCodeSet(const std::string& filename) {
        cv::FileStorage fs(filename, cv::FileStorage::READ);
        if (!fs.isOpened()) {
            writeCodeSet(filename);
            LOG::error("default config not find, write config to " + filename);
        }
        else {
            fs["takePictureFlag"] >> instance().takePictureFlag;
            fs["useDebugGuiFlag"] >> instance().useDebugGuiFlag;
            fs["grayAverage"] >> instance().grayAverage;
            fs["useAutoSaveSampleMode"] >> instance().useAutoSaveSampleMode;

            LOG::debug("codeSet.takePictureFlag=" + std::to_string(instance().takePictureFlag));
            LOG::debug("codeSet.useDebugGuiFlag=" + std::to_string(instance().useDebugGuiFlag));
            LOG::debug("codeSet.grayAverage=" + std::to_string(instance().grayAverage));
            LOG::debug("codeSet.useAutoSaveSampleMode=" + std::to_string(instance().useAutoSaveSampleMode));
        }
        fs.release();
    }

    static void writeCodeSet(const std::string& filename) {
        cv::FileStorage fs(filename, cv::FileStorage::WRITE);
        //写入注释
        fs.writeComment("代码设置\n");
        fs.writeComment("接相机下显示图像");
        fs << "useDebugGuiFlag" << instance().useDebugGuiFlag;
        fs.writeComment("代码运行时录像");
        fs << "takePictureFlag" << instance().takePictureFlag;
        fs.writeComment("自动曝光的平均灰度");
        fs << "grayAverage" << instance().grayAverage;
        fs.writeComment("0:停止保存 1:自动保存  默认:0");
        fs << "useAutoSaveSampleMode" << instance().useAutoSaveSampleMode;
        //销毁
        fs.release();
    }

    static bool getUseDebugGuiFlag() {
        return instance().useDebugGuiFlag;
    }

    static bool getTakePictureFlag() {
        return instance().takePictureFlag;
    }

    static int getGrayAverage() {
        return instance().grayAverage;
    }

    static SaveSampleMode getUseAutoSaveSampleMode() {
        return (SaveSampleMode)instance().useAutoSaveSampleMode;
    }
};

#endif // !_CONF_H_
