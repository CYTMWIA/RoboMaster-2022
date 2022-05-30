#include "DahuaCapture.hpp"

#include "/opt/HuarayTech/MVviewer/include/IMVApi.h"

#include "logging/logging.hpp"

#include "cstdio"

namespace rmcv::capture
{
    void displayDeviceInfo(IMV_DeviceList deviceInfoList)
    {
        IMV_DeviceInfo *pDevInfo = NULL;
        unsigned int cameraIndex = 0;
        char vendorNameCat[11];
        char cameraNameCat[16];

        // 打印Title行
        // Print title line
        printf("\nIdx Type Vendor     Model      S/N             DeviceUserID    IP Address    \n");
        printf("------------------------------------------------------------------------------\n");

        for (cameraIndex = 0; cameraIndex < deviceInfoList.nDevNum; cameraIndex++)
        {
            pDevInfo = &deviceInfoList.pDevInfo[cameraIndex];
            // 设备列表的相机索引  最大表示字数：3
            // Camera index in device list, display in 3 characters
            printf("%-3d", cameraIndex + 1);

            // 相机的设备类型（GigE，U3V，CL，PCIe）
            // Camera type
            switch (pDevInfo->nCameraType)
            {
            case typeGigeCamera:
                printf(" GigE");
                break;
            case typeU3vCamera:
                printf(" U3V ");
                break;
            case typeCLCamera:
                printf(" CL  ");
                break;
            case typePCIeCamera:
                printf(" PCIe");
                break;
            default:
                printf("     ");
                break;
            }

            // 制造商信息  最大表示字数：10
            // Camera vendor name, display in 10 characters
            if (strlen(pDevInfo->vendorName) > 10)
            {
                memcpy(vendorNameCat, pDevInfo->vendorName, 7);
                vendorNameCat[7] = '\0';
                strcat(vendorNameCat, "...");
                printf(" %-10.10s", vendorNameCat);
            }
            else
            {
                printf(" %-10.10s", pDevInfo->vendorName);
            }

            // 相机的型号信息 最大表示字数：10
            // Camera model name, display in 10 characters
            printf(" %-10.10s", pDevInfo->modelName);

            // 相机的序列号 最大表示字数：15
            // Camera serial number, display in 15 characters
            printf(" %-15.15s", pDevInfo->serialNumber);

            // 自定义用户ID 最大表示字数：15
            // Camera user id, display in 15 characters
            if (strlen(pDevInfo->cameraName) > 15)
            {
                memcpy(cameraNameCat, pDevInfo->cameraName, 12);
                cameraNameCat[12] = '\0';
                strcat(cameraNameCat, "...");
                printf(" %-15.15s", cameraNameCat);
            }
            else
            {
                printf(" %-15.15s", pDevInfo->cameraName);
            }

            // GigE相机时获取IP地址
            // IP address of GigE camera
            if (pDevInfo->nCameraType == typeGigeCamera)
            {
                printf(" %s", pDevInfo->DeviceSpecificInfo.gigeDeviceInfo.ipAddress);
            }

            printf("\n");
        }

        return;
    }

    class DahuaCapture::Impl
    {
    private:
        IMV_HANDLE dev_;

    public:
        Impl()
        {
            IMV_DeviceList devices;
            if (IMV_EnumDevices(&devices, interfaceTypeAll) != IMV_OK)
                __LOG_ERROR_AND_EXIT("获取设备列表失败");
            if (devices.nDevNum < 1)
                __LOG_ERROR_AND_EXIT("未发现设备");
            displayDeviceInfo(devices);

            unsigned int dev_idx = 0;
            if (IMV_CreateHandle(&dev_, modeByIndex, (void *)&dev_idx) != IMV_OK)
                __LOG_ERROR_AND_EXIT("创建设备句柄失败");
            if (IMV_Open(dev_) != IMV_OK)
                __LOG_ERROR_AND_EXIT("打开相机失败");

            if (IMV_StartGrabbing(dev_) != IMV_OK)
                __LOG_ERROR_AND_EXIT("拉流失败");
        }

        ~Impl()
        {
            IMV_StopGrabbing(dev_);
            IMV_Close(dev_);
        }

        bool set_exposure_time(float time)
        {
            if (IMV_SetDoubleFeatureValue(dev_, "ExposureTime", time) == IMV_OK)
                return true;
            else
            {
                __LOG_WARNING("曝光时间设置失败");
                return false;
            }
        }

        bool set_gain(float gain)
        {
            if (IMV_SetDoubleFeatureValue(dev_, "GainRaw", gain) == IMV_OK)
                return true;
            else
            {
                __LOG_WARNING("增益设置失败");
                return false;
            }
        }

        bool set_white_balance(float red, float green, float blue)
        {
            if (   IMV_SetEnumFeatureValue(dev_, "BalanceRatioSelector", 0) == IMV_OK && IMV_SetDoubleFeatureValue(dev_, "BalanceRatio", red) == IMV_OK
                && IMV_SetEnumFeatureValue(dev_, "BalanceRatioSelector", 1) == IMV_OK && IMV_SetDoubleFeatureValue(dev_, "BalanceRatio", green) == IMV_OK
                && IMV_SetEnumFeatureValue(dev_, "BalanceRatioSelector", 2) == IMV_OK && IMV_SetDoubleFeatureValue(dev_, "BalanceRatio", blue) == IMV_OK)
            {
                return true;
            }
            else
            {
                __LOG_WARNING("白平衡设置失败\n");
                return false;
            }
        }

        cv::Mat next()
        {
            IMV_Frame frame;
            cv::Mat img;

            if (IMV_GetFrame(dev_, &frame, 500)!=IMV_OK)
                throw std::runtime_error("获取图像失败");
            
            img.create(frame.frameInfo.height, frame.frameInfo.width, CV_8UC3);
            IMV_PixelConvertParam param;
            param.nWidth = frame.frameInfo.width;
            param.nHeight = frame.frameInfo.height;
            param.ePixelFormat = frame.frameInfo.pixelFormat;
            param.pSrcData = frame.pData;
            param.nSrcDataLen = frame.frameInfo.size;
            param.nPaddingX = frame.frameInfo.paddingX;
            param.nPaddingY = frame.frameInfo.paddingY;
            param.eBayerDemosaic = demosaicNearestNeighbor;
            param.eDstPixelFormat = gvspPixelBGR8;
            param.pDstBuf = img.data;
            param.nDstBufSize = param.nWidth*param.nHeight*3;
            param.nDstDataLen = param.nWidth*param.nHeight*3;
            if (IMV_PixelConvert(dev_, &param)!=IMV_OK)
                __LOG_WARNING("转换像素失败");

            if (IMV_ReleaseFrame(dev_, &frame)!=IMV_OK)
                throw std::runtime_error("释放图像缓存失败");

            return img;
        }
    };

    DahuaCapture::DahuaCapture() : pimpl{std::make_unique<Impl>()} {}
    DahuaCapture::~DahuaCapture() = default;

    bool DahuaCapture::set_exposure_time(float time)
    {
        return pimpl->set_exposure_time(time);
    }

    bool DahuaCapture::set_gain(float gain)
    {
        return pimpl->set_gain(gain);
    }

    bool DahuaCapture::set_white_balance(float red, float green, float blue)
    {
        return pimpl->set_white_balance(red, green, blue);
    }

    cv::Mat DahuaCapture::next()
    {
        return pimpl->next();
    }
}