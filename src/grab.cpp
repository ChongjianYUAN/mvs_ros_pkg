#include "MvCameraControl.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <stdio.h>

using namespace std;

bool is_undistorted = true;
unsigned int g_nPayloadSize = 0;
std::string ExposureAutoStr[3] = {"Off", "Once", "Continues"};
std::string GainAutoStr[3] = {"Off", "Once", "Continues"};

void setParams(void *handle, const std::string &params_file) {
  cv::FileStorage Params(params_file, cv::FileStorage::READ);
  if (!Params.isOpened()) {
    string msg = "Failed to open settings file at:" + params_file;
    ROS_ERROR(msg.c_str());
    exit(-1);
  }
  int ExposureAuto = Params["ExposureAuto"];
  int ExposureTimeLower = Params["AutoExposureTimeLower"];
  int ExposureTimeUpper = Params["AutoExposureTimeUpper"];
  int GainAuto = Params["GainAuto"];
  int GammaSelector = Params["GammaSelector"];
  float FrameRate = Params["FrameRate"];
  int nRet;
  nRet = MV_CC_SetEnumValue(handle, "ExposureAuto", ExposureAuto);
  if (MV_OK == nRet) {
    std::string msg = "Set Exposure Auto: " + ExposureAutoStr[ExposureAuto];
    ROS_INFO(msg.c_str());
  } else {
    ROS_ERROR("Fail to set Exposure auto mode");
  }
  nRet = MV_CC_SetAutoExposureTimeLower(handle, ExposureTimeLower);
  if (MV_OK == nRet) {
    std::string msg =
        "Set Exposure Time Lower: " + std::to_string(ExposureTimeLower) + "ms";
    ROS_INFO(msg.c_str());
  } else {
    ROS_ERROR("Fail to set Exposure Time Lower");
  }
  nRet = MV_CC_SetAutoExposureTimeUpper(handle, ExposureTimeUpper);
  if (MV_OK == nRet) {
    std::string msg =
        "Set Exposure Time Upper: " + std::to_string(ExposureTimeUpper) + "ms";
    ROS_INFO(msg.c_str());
  } else {
    ROS_ERROR("Fail to set Exposure Time Upper");
  }
  nRet = MV_CC_SetEnumValue(handle, "GainAuto", GainAuto);
  if (MV_OK == nRet) {
    std::string msg = "Set Gain Auto: " + GainAutoStr[GainAuto];
    ROS_INFO(msg.c_str());
  } else {
    ROS_ERROR("Fail to set Gain auto mode");
  }
  nRet = MV_CC_SetFrameRate(handle, FrameRate);
  if (MV_OK == nRet) {
    std::string msg = "Set Frame Rate: " + std::to_string(FrameRate) + "hz";
    ROS_INFO(msg.c_str());
  } else {
    ROS_ERROR("Fail to set Frame Rate");
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "grab");
  std::string params_file = std::string(argv[1]);
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("mvs_camera/image", 1);
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
  ros::Rate loop_rate(10);
  while (ros::ok()) {
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

    nRet = MV_CC_SetEnumValue(handle, "TriggerMode", 0);
    if (MV_OK != nRet) {
      printf("Set Trigger Mode fail\n");
      break;
    }

    nRet = MV_CC_SetFloatValue(handle, "Gain", 18);
    if (nRet != MV_OK) {
      printf("Gain setting can't work.\n");
      // break;
    }

    MVCC_INTVALUE stParam;
    memset(&stParam, 0, sizeof(MVCC_INTVALUE));
    nRet = MV_CC_GetIntValue(handle, "PayloadSize", &stParam);
    if (MV_OK != nRet) {
      printf("Get PayloadSize fail\n");
      break;
    }
    g_nPayloadSize = stParam.nCurValue;

    // nRet = MV_CC_SetEnumValue(handle, "PixelFormat", 0x02180014);
    // if(nRet != MV_OK)
    // {
    //     printf("My setting can't work.");
    //     break;
    // }

    nRet = MV_CC_SetEnumValue(handle, "PixelFormat", 0x02180014);
    if (nRet != MV_OK) {
      printf("Pixel setting can't work.");
      break;
    }

    setParams(handle, params_file);

    nRet = MV_CC_StartGrabbing(handle);
    if (MV_OK != nRet) {
      printf("Start Grabbing fail.\n");
      break;
    }

    MV_FRAME_OUT_INFO_EX stImageInfo = {0};
    unsigned char *pData =
        (unsigned char *)malloc(sizeof(unsigned char) * (g_nPayloadSize));
    // cv::namedWindow("camera", CV_WINDOW_KEEPRATIO);
    // cv::namedWindow("camera2", CV_WINDOW_KEEPRATIO);

    // cout << "Give a headname" << endl;
    // string name;
    // cin >> name;
    // name = "/home/dji/catkin_ws/src/opencv_exercise/pic/" + name;
    // cv::FileStorage fs(name, cv::FileStorage::WRITE);

    nRet =
        MV_CC_GetImageForBGR(handle, pData, g_nPayloadSize, &stImageInfo, 100);
    if (MV_OK != nRet) {
      printf("No data");
      std::free(pData);
      pData = NULL;
      break;
    }

    ROS_INFO("Open camera sucessfully!");

    cv::Size imageSize;
    imageSize.height = stImageInfo.nHeight;
    imageSize.width = stImageInfo.nWidth;

    cv::Mat view, rview, map1, map2;
    cv::initUndistortRectifyMap(
        cameraMatrix, distCoeffs, cv::Mat(),
        cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1,
                                      imageSize, 0),
        imageSize, CV_16SC2, map1, map2);

    while (ros::ok()) {
      memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));
      if (pData == NULL) {
        printf("Allocate memory failed.\n");
        break;
      }

      nRet = MV_CC_GetImageForBGR(handle, pData, g_nPayloadSize, &stImageInfo,
                                  100);
      if (MV_OK != nRet) {
        printf("No data");
        std::free(pData);
        pData = NULL;
        break;
      }

      cv::Mat srcImage, calibration;
      srcImage =
          cv::Mat(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC3, pData);
      sensor_msgs::ImagePtr msg =
          cv_bridge::CvImage(std_msgs::Header(), "bgr8", srcImage).toImageMsg();
      pub.publish(msg);
      ros::spinOnce();
      loop_rate.sleep();
      //   if (is_undistorted) {
      //     imshow("camera", srcImage);
      //     remap(srcImage, srcImage, map1, map2, cv::INTER_LINEAR);
      //     imshow("camera2", srcImage);
      //   } else {
      //     cv::imshow("camera", srcImage);
      //   }

      srcImage.release();
    }
    // fs.release();
    free(pData);
    nRet = MV_CC_StopGrabbing(handle);
    nRet = MV_CC_CloseDevice(handle);
    nRet = MV_CC_DestroyHandle(handle);
    break;
  }

  if (nRet != MV_OK) {
    if (handle != NULL) {
      MV_CC_DestroyHandle(handle);
      handle = NULL;
    }
  }

  return 0;
}
