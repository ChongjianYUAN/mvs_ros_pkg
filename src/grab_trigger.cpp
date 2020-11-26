#include "MvCameraControl.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <pthread.h>
#include <ros/ros.h>
#include <stdio.h>
#include <unistd.h>
using namespace std;

unsigned int g_nPayloadSize = 0;
bool exit_flag = true;
uint64_t strobe_time = 0;
image_transport::Publisher pub_opencv;

void strobeCbk(const std_msgs::Header::ConstPtr &msg) {
  // printf("----%.9lf------\n", msg->stamp.toSec());
  strobe_time = msg->stamp.toNSec();
}

void __stdcall ImageCallBackEx(unsigned char *pData,
                               MV_FRAME_OUT_INFO_EX *pFrameInfo, void *pUser) {
  static uint last_cnt = pFrameInfo->nTriggerIndex;
  static uint64_t cur_strobe = strobe_time;
  static uint64_t cur_framehead =
      (((uint64_t)pFrameInfo->nDevTimeStampHigh << 32) |
       (pFrameInfo->nDevTimeStampLow));

  if (last_cnt != pFrameInfo->nTriggerIndex) {
    cur_strobe = strobe_time + 1e9;
    cur_framehead = (((uint64_t)pFrameInfo->nDevTimeStampHigh << 32) |
                     (pFrameInfo->nDevTimeStampLow));
  }
  // printf("%.9lf\n", (cur_strobe +
  // (((uint64_t)pFrameInfo->nDevTimeStampHigh<<32) |
  // (pFrameInfo->nDevTimeStampLow)) - cur_framehead)/1000000000.0 );
  uint64_t frame_time =
      (cur_strobe + (((uint64_t)pFrameInfo->nDevTimeStampHigh << 32) |
                     (pFrameInfo->nDevTimeStampLow)) -
       cur_framehead);
  last_cnt = pFrameInfo->nTriggerIndex;

  cv::Mat srcImage;

  unsigned int nWidth = pFrameInfo->nWidth;
  unsigned int nHeight = pFrameInfo->nHeight;
  for (unsigned int j = 0; j < nHeight; j++) {
    for (unsigned int i = 0; i < nWidth; i++) {
      unsigned char red = pData[j * (nWidth * 3) + i * 3];
      pData[j * (nWidth * 3) + i * 3] = pData[j * (nWidth * 3) + i * 3 + 2];
      pData[j * (nWidth * 3) + i * 3 + 2] = red;
    }
  }

  srcImage = cv::Mat(nHeight, nWidth, CV_8UC3, pData);
  sensor_msgs::ImagePtr msg =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", srcImage).toImageMsg();

  msg->header.stamp.nsec = frame_time % 1000000000;
  msg->header.stamp.sec = frame_time / 1000000000;
  msg->header.frame_id = "camera_init";
  pub_opencv.publish(msg);

  // if(pFrameInfo)
  // {
  //   printf("GetOneFrame, frame num[%d] trigger cnt[%d] dev[%lu]
  //   time[%u][%u][%u]\n",
  //         pFrameInfo->nFrameNum, pFrameInfo->nTriggerIndex,
  //         (((uint64_t)pFrameInfo->nDevTimeStampHigh<<32) |
  //         (pFrameInfo->nDevTimeStampLow)),
  //         pFrameInfo->nSecondCount, pFrameInfo->nCycleCount,
  //         pFrameInfo->nCycleOffset);
  // }
}

static void *WorkThread(void *pUser) {
  while (exit_flag) {
  }

  return NULL;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "grab_trigger");
  ros::NodeHandle n;
  ros::Subscriber sub_strobe = n.subscribe("/strobe_time", 1000, strobeCbk);
  image_transport::ImageTransport it(n);
  pub_opencv = it.advertise("strobe_image", 1);

  cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
  cameraMatrix.at<double>(0, 0) = 1.730735067136013e+03;
  cameraMatrix.at<double>(0, 1) = -0.000682525720977;
  cameraMatrix.at<double>(0, 2) = 1.515012142085100e+03;
  cameraMatrix.at<double>(1, 1) = 1.730530820356212e+03;
  cameraMatrix.at<double>(1, 2) = 1.044575428820981e+03;

  cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64F);
  distCoeffs.at<double>(0, 0) = -0.095982349277083;
  distCoeffs.at<double>(1, 0) = 0.090204555257461;
  distCoeffs.at<double>(2, 0) = 0.001075320356832;
  distCoeffs.at<double>(3, 0) = -0.001243809361172;
  distCoeffs.at<double>(4, 0) = 0;

  int nRet = MV_OK;
  void *handle = NULL;

  while (1) {
    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

    nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
    if (MV_OK != nRet) {
      printf("Enum Devices fail!");
      break;
    }

    if (stDeviceList.nDeviceNum == 0) {
      printf("No Camera.\n");
      break;
    }

    nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[0]);
    if (MV_OK != nRet) {
      printf("Create Handle fail");
      break;
    }

    nRet = MV_CC_OpenDevice(handle);
    if (MV_OK != nRet) {
      printf("Open Device fail\n");
      break;
    }

    nRet = MV_CC_SetBoolValue(handle, "AcquisitionFrameRateEnable", false);
    if (MV_OK != nRet) {
      printf("set AcquisitionFrameRateEnable fail! nRet [%x]\n", nRet);
      break;
    }

    MVCC_INTVALUE stParam;
    memset(&stParam, 0, sizeof(MVCC_INTVALUE));
    nRet = MV_CC_GetIntValue(handle, "PayloadSize", &stParam);
    if (MV_OK != nRet) {
      printf("Get PayloadSize fail\n");
      break;
    }
    g_nPayloadSize = stParam.nCurValue;

    nRet = MV_CC_SetEnumValue(handle, "PixelFormat", 0x02180014);
    if (nRet != MV_OK) {
      printf("Pixel setting can't work.");
      break;
    }

    nRet = MV_CC_RegisterImageCallBackEx(handle, ImageCallBackEx, handle);
    if (MV_OK != nRet) {
      printf("MV_CC_RegisterImageCallBackEx fail! nRet [%x]\n", nRet);
      break;
    }

    nRet = MV_CC_StartGrabbing(handle);
    if (MV_OK != nRet) {
      printf("Start Grabbing fail.\n");
      break;
    }

    pthread_t nThreadID;
    nRet = pthread_create(&nThreadID, NULL, WorkThread, handle);
    if (nRet != 0) {
      printf("thread create failed.ret = %d\n", nRet);
      break;
    }
    ros::spin();
    exit_flag = false;

    nRet = MV_CC_StopGrabbing(handle);
    if (MV_OK != nRet) {
      printf("MV_CC_StopGrabbing fail! nRet [%x]\n", nRet);
      break;
    }

    nRet = MV_CC_CloseDevice(handle);
    if (MV_OK != nRet) {
      printf("MV_CC_CloseDevice fail! nRet [%x]\n", nRet);
      break;
    }

    nRet = MV_CC_DestroyHandle(handle);
    if (MV_OK != nRet) {
      printf("MV_CC_DestroyHandle fail! nRet [%x]\n", nRet);
      break;
    }

    break;
  }

  if (nRet != MV_OK) {
    if (handle != NULL) {
      MV_CC_DestroyHandle(handle);
      handle = NULL;
    }
  }
  pub_opencv.shutdown();
  return 0;
}
