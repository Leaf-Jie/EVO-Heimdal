#ifndef _CONF_H_
#define _CONF_H_

#include "tool/filePath.h"

//�������
struct CameraConfiguration {
    double gainValue = 1.5;                       //����
    int exposureValue = 5000;                   //�ع�ʱ��
    double gammaValue = 0.6;                      //gamma
};

class CameraConfigurationFactory {
private:
    static CameraConfigurationFactory& instance() {
        static CameraConfigurationFactory cameraConfigurationFactory;
        return cameraConfigurationFactory;
    }

    /**
     * �������ļ���д��һ��������ýڵ�
     * @param cameraConfiguration ��д��Ľڵ�
     * @param name �ڵ�����
     * @param comment ��д���ע��
     * @param fs �����ļ�ָ��
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
     * �������ļ��ж���һ��������ýڵ�
     * @param cameraConfiguration ������Ľڵ�
     * @param name �ڵ�����
     * @param fs �����ļ�ָ��
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
        //д��ע��
        fs.writeComment("�������");
        writeCameraConfigurationStruct(instance().cameraConfiguration, "CameraConfiguration", "�������", fs);
        //����
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
    bool useAutoSaveSampleMode = 0;  //0:ֹͣ���� 1:�Զ�����    Ĭ��:0

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
        //д��ע��
        fs.writeComment("��������\n");
        fs.writeComment("���������ʾͼ��");
        fs << "useDebugGuiFlag" << instance().useDebugGuiFlag;
        fs.writeComment("��������ʱ¼��");
        fs << "takePictureFlag" << instance().takePictureFlag;
        fs.writeComment("�Զ��ع��ƽ���Ҷ�");
        fs << "grayAverage" << instance().grayAverage;
        fs.writeComment("0:ֹͣ���� 1:�Զ�����  Ĭ��:0");
        fs << "useAutoSaveSampleMode" << instance().useAutoSaveSampleMode;
        //����
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
