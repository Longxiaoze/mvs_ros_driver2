#include "MvCameraControl.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/msg/image.hpp"
#include <chrono>
#include <fcntl.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <pthread.h>
#include <rclcpp/rclcpp.hpp>
#include <signal.h>
#include <stdio.h>
#include <sys/ipc.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

// 日志输出 port to ROS2
#define ROS_INFO(...) RCLCPP_INFO(rclcpp::get_logger("mvs_driver"), __VA_ARGS__)
#define ROS_ERROR(...)                                                         \
  RCLCPP_ERROR(rclcpp::get_logger("mvs_driver"), __VA_ARGS__)
#define ROS_WARN(...) RCLCPP_WARN(rclcpp::get_logger("mvs_driver"), __VA_ARGS__)
#define ROS_DEBUG(...)                                                         \
  RCLCPP_DEBUG(rclcpp::get_logger("mvs_driver"), __VA_ARGS__)

using namespace std;

struct time_stamp {
  int64_t high;
  int64_t low;
};
time_stamp *pointt;

enum PixelFormat : unsigned int {
  RGB8 = 0x02180014,
  BayerRG8 = 0x01080009,
  BayerRG12Packed = 0x010C002B,
  BayerGB12Packed = 0x010C002C,
  BayerGB8 = 0x0108000A
};

// unsigned int g_nPayloadSize = 0;
bool is_undistorted = true;
bool exit_flag = false;
int width, height;
rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub;
std::vector<PixelFormat> PIXEL_FORMAT = {RGB8, BayerRG8, BayerRG12Packed,
                                         BayerGB12Packed, BayerGB8};
std::string ExposureAutoStr[3] = {"Off", "Once", "Continues"};
std::string GammaSlectorStr[3] = {"User", "sRGB", "Off"};
std::string GainAutoStr[3] = {"Off", "Once", "Continues"};
float image_scale = 0.0;
int trigger_enable = 1;

bool PrintDeviceInfo(MV_CC_DEVICE_INFO *pstMVDevInfo) {
  if (NULL == pstMVDevInfo) {
    ROS_ERROR("The Pointer of pstMVDevInfo is NULL!");
    return false;
  }
  if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE) {
    int nIp1 =
        ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
    int nIp2 =
        ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
    int nIp3 =
        ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
    int nIp4 = (pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);

    ROS_INFO("[GigE] device");
    ROS_INFO("Device Model Name: %s",
             pstMVDevInfo->SpecialInfo.stGigEInfo.chModelName);
    ROS_INFO("CurrentIp: %d.%d.%d.%d", nIp1, nIp2, nIp3, nIp4);
    ROS_INFO("SerialNumber: %s",
             pstMVDevInfo->SpecialInfo.stGigEInfo.chSerialNumber);
  } else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE) {
    ROS_INFO("[USB] device");
    ROS_INFO("Device Model Name: %s",
             pstMVDevInfo->SpecialInfo.stUsb3VInfo.chModelName);
    ROS_INFO("SerialNumber: %s",
             pstMVDevInfo->SpecialInfo.stUsb3VInfo.chSerialNumber);
  } else {
    ROS_WARN("Not support.");
  }
  return true;
}

void setParams(void *handle, const std::string &params_file) {
  cv::FileStorage Params(params_file, cv::FileStorage::READ);
  if (!Params.isOpened()) {
    string msg = "Failed to open settings file at:" + params_file;
    ROS_ERROR(msg.c_str());
    exit(-1);
  }
  image_scale = Params["image_scale"];
  if (image_scale < 0.1)
    image_scale = 1;
  int ExposureTimeLower = Params["AutoExposureTimeLower"];
  int ExposureTimeUpper = Params["AutoExposureTimeUpper"];
  int ExposureTime = Params["ExposureTime"];
  int ExposureAutoMode = Params["ExposureAutoMode"];
  int GainAuto = Params["GainAuto"];
  float Gain = Params["Gain"];
  float Gamma = Params["Gamma"];
  int GammaSlector = Params["GammaSelector"];
  int nRet;

  // 设置曝光模式
  nRet = MV_CC_SetExposureAutoMode(handle, ExposureAutoMode);
  std::string msg =
      "Set ExposureAutoMode: " + ExposureAutoStr[ExposureAutoMode];

  if (MV_OK == nRet) {
    ROS_INFO(msg.c_str());
  } else {
    if (ExposureAutoMode == 2) {
      ROS_WARN("Fail to set Exposure Auto Mode to Continues");
    } else {
      ROS_INFO(msg.c_str());
    }
  }

  // 如果是自动曝光
  if (ExposureAutoMode == 2) {
    nRet = MV_CC_SetAutoExposureTimeLower(handle, ExposureTimeLower);
    if (MV_OK == nRet) {
      std::string msg =
          "Set Exposure Time Lower: " + std::to_string(ExposureTimeLower) +
          "us";
      ROS_INFO(msg.c_str());
    } else {
      ROS_ERROR("Fail to set Exposure Time Lower");
    }
    nRet = MV_CC_SetAutoExposureTimeUpper(handle, ExposureTimeUpper);
    if (MV_OK == nRet) {
      std::string msg =
          "Set Exposure Time Upper: " + std::to_string(ExposureTimeUpper) +
          "us";
      ROS_INFO(msg.c_str());
    } else {
      ROS_ERROR("Fail to set Exposure Time Upper");
    }
  }

  // 如果是固定曝光
  if (ExposureAutoMode == 0) {
    nRet = MV_CC_SetExposureTime(handle, ExposureTime);
    if (MV_OK == nRet) {
      std::string msg =
          "Set Exposure Time: " + std::to_string(ExposureTime) + "us";
      ROS_INFO(msg.c_str());
    } else {
      ROS_ERROR("Fail to set Exposure Time");
    }
  }

  nRet = MV_CC_SetEnumValue(handle, "GainAuto", GainAuto);

  if (MV_OK == nRet) {
    std::string msg = "Set Gain Auto: " + GainAutoStr[GainAuto];
    ROS_INFO(msg.c_str());
  } else {
    ROS_ERROR("Fail to set Gain auto mode");
  }

  if (GainAuto == 0) {
    nRet = MV_CC_SetGain(handle, Gain);
    if (MV_OK == nRet) {
      std::string msg = "Set Gain: " + std::to_string(Gain);
      ROS_INFO(msg.c_str());
    } else {
      ROS_ERROR("Fail to set Gain");
    }
  }

  nRet = MV_CC_SetGammaSelector(handle, GammaSlector);
  if (MV_OK == nRet) {
    std::string msg = "Set GammaSlector: " + GammaSlectorStr[GammaSlector];
    ROS_INFO(msg.c_str());
  } else {
    ROS_ERROR("Fail to set GammaSlector");
  }

  nRet = MV_CC_SetGamma(handle, Gamma);
  if (MV_OK == nRet) {
    std::string msg = "Set Gamma: " + std::to_string(Gamma);
    ROS_INFO(msg.c_str());
  } else {
    ROS_ERROR("Fail to set Gamma");
  }
}

void SignalHandler(int signal) {
  if (signal == SIGINT) { // 捕捉 Ctrl + C 触发的 SIGINT 信号
    fprintf(stderr, "\nReceived Ctrl+C, exiting...\n");
    exit_flag = true; // 设置退出标志
  }
}

void SetupSignalHandler() {
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = SignalHandler; // 设置处理函数
  sigemptyset(&sigIntHandler.sa_mask);      // 清空信号屏蔽集
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);
}

static void *WorkThread(void *pUser) {
  int nRet = MV_OK;

  unsigned char *pDataForRGB = NULL;
  MVCC_INTVALUE stParam;
  memset(&stParam, 0, sizeof(MVCC_INTVALUE));
  nRet = MV_CC_GetIntValue(pUser, "PayloadSize", &stParam);
  if (MV_OK != nRet) {
    ROS_ERROR("Get PayloadSize fail! nRet [0x%x]", nRet);
    return NULL;
  }
  ROS_INFO("Get PayloadSize success! val [%d]", stParam.nCurValue);

  // MV_FRAME_OUT_INFO_EX stImageInfo = {0};
  MV_CC_PIXEL_CONVERT_PARAM stConvertParam = {0};
  MV_FRAME_OUT stImageInfo = {0};
  MV_CC_IMAGE stImage = {0};

  // memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT));
  // MV_CC_PIXEL_CONVERT_PARAM stConvertParam = {0};

  // unsigned char *pData =
  //     (unsigned char *)malloc(sizeof(unsigned char) * stParam.nCurValue * 3);
  // unsigned char *pDataForBGR =
  //     (unsigned char *)malloc(sizeof(unsigned char) * stParam.nCurValue * 3);

  // if (pData == nullptr || pDataForBGR == nullptr) {
  //   ROS_ERROR("Memory allocation failed!");
  //   if (pData)
  //     free(pData);
  //   if (pDataForBGR)
  //     free(pDataForBGR);
  //   return nullptr;
  // }

  ROS_INFO("Capture loop start.");
  while (!exit_flag && rclcpp::ok()) {

    // nRet = MV_CC_GetOneFrameTimeout(pUser, pData, stParam.nCurValue * 3,
    //                                 &stImageInfo, 1000);

    nRet = MV_CC_GetImageBuffer(pUser, &stImageInfo, 10000);

    if (nRet == MV_OK) {

      rclcpp::Time rcv_time;
      if (trigger_enable && pointt != MAP_FAILED && pointt->low != 0) {
        // 触发模式
        // 赋值共享内存中的时间戳给相机帧
        int64_t b = pointt->low;
        double time_pc = b / 1000000000.0;
        rcv_time =
            rclcpp::Time(static_cast<int64_t>(time_pc * 1e9)); // 转换为纳
      } else {
        // 自动模式 使用 ROS 系统时钟 // TODO 确认此处是否会产生影响
        rcv_time = rclcpp::Clock(RCL_SYSTEM_TIME).now();
      }

      std::string debug_msg;
      debug_msg = "GetOneFrame,nFrameNum[" +
                  std::to_string(stImageInfo.stFrameInfo.nFrameNum) +
                  "], FrameTime:" + std::to_string(rcv_time.seconds());
      ROS_DEBUG(debug_msg.c_str());

      pDataForRGB = (unsigned char *)malloc(
          stImageInfo.stFrameInfo.nExtendWidth *
              stImageInfo.stFrameInfo.nExtendHeight * 4 +
          2048);
      if (NULL == pDataForRGB) {
        ROS_ERROR("pDataForRGB is null");
        break;
      }

      stConvertParam.nWidth = stImageInfo.stFrameInfo.nExtendWidth;
      stConvertParam.nHeight = stImageInfo.stFrameInfo.nExtendHeight;
      stConvertParam.pSrcData = stImageInfo.pBufAddr;
      stConvertParam.nSrcDataLen = stImageInfo.stFrameInfo.nFrameLenEx;
      stConvertParam.enSrcPixelType = stImageInfo.stFrameInfo.enPixelType;
      stConvertParam.enDstPixelType = PixelType_Gvsp_RGB8_Packed;
      stConvertParam.pDstBuffer = pDataForRGB;
      stConvertParam.nDstBufferSize =
          stImageInfo.stFrameInfo.nExtendWidth *
              stImageInfo.stFrameInfo.nExtendHeight * 4 +
          2048;

      nRet = MV_CC_ConvertPixelType(pUser, &stConvertParam);

      // stConvertParam.nWidth = stImageInfo.stFrameInfo.nWidth;
      // stConvertParam.nHeight = stImageInfo.stFrameInfo.nHeight;
      // stConvertParam.pSrcData = pData;
      // stConvertParam.nSrcDataLen = stImageInfo.stFrameInfo.nFrameLen;
      // stConvertParam.enSrcPixelType = stImageInfo.stFrameInfo.enPixelType;
      // stConvertParam.enDstPixelType = PixelType_Gvsp_RGB8_Packed;
      // stConvertParam.pDstBuffer = pDataForBGR;
      // stConvertParam.nDstBufferSize = stImageInfo.stFrameInfo.nFrameLen;
      // nRet = MV_CC_ConvertPixelType(pUser, &stConvertParam);
      if (MV_OK != nRet) {
        ROS_WARN(
            "MV_CC_ConvertPixelType failed! nRet [%x], skipping this frame",
            nRet);
        continue;
      }
      cv::Mat srcImage;
      srcImage = cv::Mat(stImageInfo.stFrameInfo.nHeight,
      stImageInfo.stFrameInfo.nWidth, CV_8UC3,
                         pDataForRGB);

      // // cv::Mat srcImage;
      // // srcImage = cv::Mat(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC3,
      // // pData);
      ROS_INFO("GetOneFrame, Width[%d], Height[%d], nFrameNum[%d]",
               stImageInfo.stFrameInfo.nExtendWidth,
               stImageInfo.stFrameInfo.nExtendHeight,
               stImageInfo.stFrameInfo.nFrameNum);
      MV_CC_FreeImageBuffer(pUser, &stImageInfo);
      // usleep(100000);

      stImage.nWidth = stImageInfo.stFrameInfo.nExtendWidth;
      stImage.nHeight = stImageInfo.stFrameInfo.nExtendHeight;
      stImage.enPixelType = stImageInfo.stFrameInfo.enPixelType;
      stImage.pImageBuf = stImageInfo.pBufAddr;
      stImage.nImageLen = stImageInfo.stFrameInfo.nFrameLenEx;

      if (image_scale > 0.0) {
        cv::resize(
            srcImage, srcImage,
            cv::Size(srcImage.cols * image_scale, srcImage.rows *
            image_scale), cv::INTER_LINEAR);
      } else {
        ROS_WARN("Invalid image_scale: %f. Skipping resize.", image_scale);
      }
      sensor_msgs::msg::Image msg;
      msg.header.stamp = rcv_time;
      msg.height = srcImage.rows;
      msg.width = srcImage.cols;
      msg.encoding = "rgb8";
      msg.is_bigendian = false;
      msg.step = srcImage.step;
      msg.data.assign(srcImage.data,
                      srcImage.data + srcImage.total() *
                      srcImage.elemSize());
      // msg.header.stamp = rclcpp::Clock().now();

      pub->publish(msg);
    } else {
      ROS_WARN("Capture timeout, retrying...");
    }
  }

  // if (pData) {
  //   free(pData);
  //   pData = nullptr;
  // }

  // if (pDataForBGR) {
  //   free(pDataForBGR);
  //   pDataForBGR = nullptr;
  // }

  return 0;
}

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);
  std::string params_file = std::string(argv[1]);
  // cv::FileStorage Params(params_file, cv::FileStorage::READ);
  // trigger_enable = Params["TriggerEnable"];
  // std::string expect_serial_number = Params["SerialNumber"];
  // std::string pub_topic = Params["TopicName"];
  // int PixelFormat = Params["PixelFormat"];

  int nRet = MV_OK;
  void *handle = NULL;
  rclcpp::Rate loop_rate(10);
  cv::FileStorage Params(params_file, cv::FileStorage::READ);
  if (!Params.isOpened()) {
    string msg = "Failed to open settings file at:" + params_file;
    ROS_ERROR(msg.c_str());
    exit(-1);
  }
  ROS_INFO("Load settings from file: %s", params_file.c_str());
  trigger_enable = Params["TriggerEnable"];
  std::string expect_serial_number = Params["SerialNumber"];
  std::string pub_topic = Params["TopicName"];
  int PixelFormat = Params["PixelFormat"];

  auto node = rclcpp::Node::make_shared("mvs_trigger");
  pub = node->create_publisher<sensor_msgs::msg::Image>(pub_topic, 10);

  const char *user_name = getlogin();
  std::string path_for_time_stamp =
      "/home/" + std::string(user_name) + "/timeshare";
  const char *shared_file_name = path_for_time_stamp.c_str();
  int fd = open(shared_file_name, O_RDWR);
  pointt = (time_stamp *)mmap(NULL, sizeof(time_stamp), PROT_READ | PROT_WRITE,
                              MAP_SHARED, fd, 0);

  SetupSignalHandler();

  MV_CC_DEVICE_INFO_LIST stDeviceList;
  memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

  nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
  if (MV_OK != nRet) {
    ROS_ERROR("MV_CC_EnumDevices fail! nRet [%x]", nRet);
    return -1;
  }

  if (stDeviceList.nDeviceNum > 0) {
    for (int i = 0; i < stDeviceList.nDeviceNum; i++) {
      ROS_INFO("[device %d]:", i);
      MV_CC_DEVICE_INFO *pDeviceInfo = stDeviceList.pDeviceInfo[i];
      if (pDeviceInfo == NULL) {
        ROS_ERROR("Device info is NULL for device %d", i);
        return -1;
      }
      PrintDeviceInfo(pDeviceInfo);
    }
  } else {
    ROS_ERROR("Find No Devices!");
    return -1;
  }

  bool find_expect_camera = false;
  unsigned int nIndex = 0;

  if (stDeviceList.nDeviceNum > 1) {
    if (expect_serial_number.empty()) {
      ROS_ERROR("Expected serial number is empty!");
      return -1;
    }
    for (int i = 0; i < stDeviceList.nDeviceNum; i++) {
      if (stDeviceList.pDeviceInfo[i] == NULL) {
        ROS_ERROR("Device info is NULL for device %d", i);
        continue;
      }

      std::string serial_number;
      if (stDeviceList.pDeviceInfo[i]->nTLayerType == MV_USB_DEVICE) {
        serial_number =
            std::string((char *)stDeviceList.pDeviceInfo[i]
                            ->SpecialInfo.stUsb3VInfo.chSerialNumber);
      } else if (stDeviceList.pDeviceInfo[i]->nTLayerType == MV_GIGE_DEVICE) {
        serial_number =
            std::string((char *)stDeviceList.pDeviceInfo[i]
                            ->SpecialInfo.stGigEInfo.chSerialNumber);
      } else {
        ROS_ERROR("Unknown device type!");
        continue;
      }
      if (serial_number.empty()) {
        ROS_ERROR("Serial number is empty for device %d", i);
        continue;
      }
      if (expect_serial_number == serial_number) {
        find_expect_camera = true;
        nIndex = i;
        break;
      }
    }
    if (!find_expect_camera) {
      std::string msg =
          "Can not find the camera with serial number " + expect_serial_number;
      ROS_ERROR(msg.c_str());
      return -1;
    }
  } else {
    nIndex = 0;
  }

  // select device and create handle
  nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[nIndex]);
  if (MV_OK != nRet) {
    ROS_ERROR("MV_CC_CreateHandle fail! nRet [%x]", nRet);
    return -1;
  }

  // open device
  nRet = MV_CC_OpenDevice(handle);
  if (MV_OK != nRet) {
    ROS_ERROR("MV_CC_OpenDevice fail! nRet [%x]", nRet);
    return -1;
  }

  nRet = MV_CC_SetBoolValue(handle, "AcquisitionFrameRateEnable", false);
  if (MV_OK != nRet) {
    ROS_ERROR("set AcquisitionFrameRateEnable fail! nRet [%x]", nRet);
    return -1;
  }

  // MVCC_INTVALUE stParam;
  // memset(&stParam, 0, sizeof(MVCC_INTVALUE));
  // nRet = MV_CC_GetIntValue(handle, "PayloadSize", &stParam);
  // if (MV_OK != nRet) {
  //   printf("Get PayloadSize fail\n");
  //   return -1;
  // }
  // g_nPayloadSize = stParam.nCurValue * 3;

  nRet = MV_CC_SetEnumValue(
      handle, "PixelFormat",
      PIXEL_FORMAT[PixelFormat]); // BayerRG8 0x01080009 RGB8 0x02180014
                                  // BayerRG12Packed 0x010C002B
  if (nRet != MV_OK) {
    ROS_ERROR("Pixel setting can't work.");
    return -1;
  }

  setParams(handle, params_file);

  // set trigger mode as on
  nRet = MV_CC_SetEnumValue(handle, "TriggerMode", trigger_enable);
  if (MV_OK != nRet) {
    ROS_ERROR("MV_CC_SetTriggerMode fail! nRet [%x]", nRet);
    return -1;
  } else {
    ROS_INFO("Set TriggerMode [%s]", trigger_enable == 0 ? "OFF" : "ON");
  }

  // set trigger source
  nRet = MV_CC_SetEnumValue(handle, "TriggerSource", MV_TRIGGER_SOURCE_LINE0);
  if (MV_OK != nRet) {
    ROS_ERROR("MV_CC_SetTriggerSource fail! nRet [%x]", nRet);
    return -1;
  }

  ROS_INFO("Finish all params set! Start grabbing...");
  nRet = MV_CC_StartGrabbing(handle);
  if (MV_OK != nRet) {
    ROS_ERROR("Start Grabbing fail.");
    return -1;
  }
  ROS_INFO("Start Grabbing Success.");

  pthread_t nThreadID;
  nRet = pthread_create(&nThreadID, NULL, WorkThread, handle);
  if (nRet != 0) {
    ROS_ERROR("thread create failed.ret = %d", nRet);
    return -1;
  }
  ROS_INFO("Start Grabbing thread Success, pid %ld", nThreadID);

  while (!exit_flag && rclcpp::ok()) {
    rclcpp::spin_some(node);
    usleep(100000);
  }

  if (nThreadID) {
    pthread_join(nThreadID, NULL);
    ROS_INFO("Worker thread joined.");
  }

  nRet = MV_CC_StopGrabbing(handle);
  if (MV_OK != nRet) {
    ROS_ERROR("MV_CC_StopGrabbing fail! nRet [%x]", nRet);
    return -1;
  }
  ROS_INFO("MV_CC_StopGrabbing success!");

  nRet = MV_CC_CloseDevice(handle);
  if (MV_OK != nRet) {
    ROS_ERROR("MV_CC_CloseDevice fail! nRet [%x]", nRet);
    return -1;
  }
  ROS_INFO("MV_CC_CloseDevice success!");

  nRet = MV_CC_DestroyHandle(handle);
  if (MV_OK != nRet) {
    ROS_ERROR("MV_CC_DestroyHandle fail! nRet [%x]", nRet);
    return -1;
  }
  ROS_INFO("MV_CC_DestroyHandle success!");

  munmap(pointt, sizeof(time_stamp));

  return 0;
}