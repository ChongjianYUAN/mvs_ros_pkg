
#ifndef _MV_CAMERA_PARAMS_H_
#define _MV_CAMERA_PARAMS_H_

#include "PixelType.h"

#ifndef __cplusplus
typedef char    bool;
#define true    1
#define false   0
#endif

// ch:设备类型定义 | en:Device Type Definition
#define MV_UNKNOW_DEVICE        0x00000000          // ch:未知设备类型，保留意义 | en:Unknown Device Type, Reserved 
#define MV_GIGE_DEVICE          0x00000001          // ch:GigE设备 | en:GigE Device
#define MV_1394_DEVICE          0x00000002          // ch:1394-a/b 设备 | en:1394-a/b Device
#define MV_USB_DEVICE           0x00000004          // ch:USB3.0 设备 | en:USB3.0 Device
#define MV_CAMERALINK_DEVICE    0x00000008          // ch:CameraLink设备 | en:CameraLink Device

typedef struct _MV_GIGE_DEVICE_INFO_
{
    unsigned int        nIpCfgOption;        
    unsigned int        nIpCfgCurrent;       //IP configuration:bit31-static bit30-dhcp bit29-lla      
    unsigned int        nCurrentIp;          //curtent  | en:         
    unsigned int        nCurrentSubNetMask;  //curtent subnet mask     
    unsigned int        nDefultGateWay;      //current gateway | en
    unsigned char       chManufacturerName[32];
    unsigned char       chModelName[32];
    unsigned char       chDeviceVersion[32];
    unsigned char       chManufacturerSpecificInfo[48];
    unsigned char       chSerialNumber[16];
    unsigned char       chUserDefinedName[16]; 

    unsigned int        nNetExport;         // ch:网口IP地址 | en:GIGE IP Address

    unsigned int        nReserved[4];

}MV_GIGE_DEVICE_INFO;

#define INFO_MAX_BUFFER_SIZE 64    //新定义256
typedef struct _MV_USB3_DEVICE_INFO_
{
    unsigned char           CrtlInEndPoint;                        // ch:控制输入端点 | en:Control input endpoint
    unsigned char           CrtlOutEndPoint;                       // ch:控制输出端点 | en:Control output endpoint
    unsigned char           StreamEndPoint;                        // ch:流端点 | en:Flow endpoint
    unsigned char           EventEndPoint;                         // ch:事件端点 | en:Event endpoint
    unsigned short          idVendor;                              // ch:供应商ID号 | en:Vendor ID Number
    unsigned short          idProduct;                             // ch:产品ID号 | en:Device ID Number
    unsigned int            nDeviceNumber;                         // ch:设备序列号  | en:Device Serial Number
    unsigned char           chDeviceGUID[INFO_MAX_BUFFER_SIZE];    // ch:设备GUID号 | en:Device GUID Number
    unsigned char           chVendorName[INFO_MAX_BUFFER_SIZE];    // ch:供应商名字 | en:Vendor Name
    unsigned char           chModelName[INFO_MAX_BUFFER_SIZE];     // ch:型号名字 | en:Model Name
    unsigned char           chFamilyName[INFO_MAX_BUFFER_SIZE];    // ch:家族名字 | en:Family Name
    unsigned char           chDeviceVersion[INFO_MAX_BUFFER_SIZE];  // ch:设备版本号 | en:Device Version
    unsigned char           chManufacturerName[INFO_MAX_BUFFER_SIZE]; // ch:制造商名字 | en:Manufacturer Name
    unsigned char           chSerialNumber[INFO_MAX_BUFFER_SIZE];      // ch:序列号 | en:Serial Number
    unsigned char           chUserDefinedName[INFO_MAX_BUFFER_SIZE];  // ch:用户自定义名字 | en:User Defined Name
    unsigned int            nbcdUSB;                               // ch:支持的USB协议 | en:Support USB Protocol
    unsigned int            nReserved[3];                             // ch:保留字节 | en:Reserved bytes
}MV_USB3_DEVICE_INFO;

typedef struct _MV_CamL_DEV_INFO_
{
    unsigned char           chPortID[INFO_MAX_BUFFER_SIZE];
    unsigned char           chModelName[INFO_MAX_BUFFER_SIZE];
    unsigned char           chFamilyName[INFO_MAX_BUFFER_SIZE];
    unsigned char           chDeviceVersion[INFO_MAX_BUFFER_SIZE];
    unsigned char           chManufacturerName[INFO_MAX_BUFFER_SIZE];
    unsigned char           chSerialNumber[INFO_MAX_BUFFER_SIZE];
    unsigned int            nReserved[38];
}MV_CamL_DEV_INFO;


// ch:设备信息 | en:Device Infomation
typedef struct _MV_CC_DEVICE_INFO_
{
    // ch:common info | en:common info 
    unsigned short      nMajorVer;
    unsigned short      nMinorVer;
    unsigned int        nMacAddrHigh;               // ch:MAC 地址 | en:MAC Address
    unsigned int        nMacAddrLow;

    unsigned int        nTLayerType;                // ch:设备传输层协议类型，e.g. MV_GIGE_DEVICE | en:Device Transport Layer Protocol Type, e.g. MV_GIGE_DEVICE

    unsigned int        nReserved[4];

    union
    {
        MV_GIGE_DEVICE_INFO stGigEInfo;
        MV_USB3_DEVICE_INFO stUsb3VInfo; 
        MV_CamL_DEV_INFO       stCamLInfo;
        // more ...
    }SpecialInfo;

}MV_CC_DEVICE_INFO;

// ch:网络传输的相关信息 | en:Network transmission information
typedef struct _MV_NETTRANS_INFO_
{
    int64_t         nReviceDataSize;    // ch:已接收数据大小  [统计StartGrabbing和StopGrabbing之间的数据量] | en:Received Data Size  [Calculate the Data Size between StartGrabbing and StopGrabbing]
    int             nThrowFrameCount;   // ch:丢帧数量 | en:Throw frame number
    unsigned int    nNetRecvFrameCount;
    int64_t         nRequestResendPacketCount;          // 请求重发包数
    int64_t         nResendPacketCount;   // 重发包数

}MV_NETTRANS_INFO;


// ch:最多支持的传输层实例个数 | en:The maximum number of supported transport layer instances
#define MV_MAX_TLS_NUM          8
// ch:最大支持的设备个数 | en:The maximum number of supported devices
#define MV_MAX_DEVICE_NUM      256

// ch:设备信息列表 | en:Device Information List
typedef struct _MV_CC_DEVICE_INFO_LIST_
{
    unsigned int            nDeviceNum;                         // ch:在线设备数量 | en:Online Device Number
    MV_CC_DEVICE_INFO*      pDeviceInfo[MV_MAX_DEVICE_NUM];     // ch:支持最多256个设备 | en:Support up to 256 devices

}MV_CC_DEVICE_INFO_LIST;


// ch:输出帧的信息 | en:Output Frame Information
typedef struct _MV_FRAME_OUT_INFO_
{
    unsigned short      nWidth;             // ch:图像宽 | en:Image Width
    unsigned short      nHeight;            // ch:图像高 | en:Image Height
    enum MvGvspPixelType     enPixelType;        // ch:像素格式 | en:Pixel Type

    unsigned int        nFrameNum;          // ch:帧号 | en:Frame Number
    unsigned int        nDevTimeStampHigh;  // ch:时间戳高32位 | en:Timestamp high 32 bits
    unsigned int        nDevTimeStampLow;   // ch:时间戳低32位 | en:Timestamp low 32 bits
    unsigned int        nReserved0;         // ch:保留，8字节对齐 | en:Reserved, 8-byte aligned
    int64_t             nHostTimeStamp;     // ch:主机生成的时间戳 | en:Host-generated timestamp

    unsigned int        nFrameLen;

    unsigned int        nLostPacket;  // 本帧丢包数
    unsigned int        nReserved[2];
}MV_FRAME_OUT_INFO;

// ch:Chunk内容 | en:The content of ChunkData
typedef struct _MV_CHUNK_DATA_CONTENT_
{
    unsigned char*  pChunkData;
    unsigned int    nChunkID;
    unsigned int    nChunkLen;

    unsigned int    nReserved[8];       // 保留

}MV_CHUNK_DATA_CONTENT;

// ch:输出帧的信息 | en:Output Frame Information
typedef struct _MV_FRAME_OUT_INFO_EX_
{
    unsigned short      nWidth;             // ch:图像宽 | en:Image Width
    unsigned short      nHeight;            // ch:图像高 | en:Image Height
    enum MvGvspPixelType     enPixelType;        // ch:像素格式 | en:Pixel Type

    unsigned int        nFrameNum;          // ch:帧号 | en:Frame Number
    unsigned int        nDevTimeStampHigh;  // ch:时间戳高32位 | en:Timestamp high 32 bits
    unsigned int        nDevTimeStampLow;   // ch:时间戳低32位 | en:Timestamp low 32 bits
    unsigned int        nReserved0;         // ch:保留，8字节对齐 | en:Reserved, 8-byte aligned
    int64_t             nHostTimeStamp;     // ch:主机生成的时间戳 | en:Host-generated timestamp

    unsigned int        nFrameLen;

    // ch:以下为chunk新增水印信息 | en:The followings are chunk add frame-specific information
    // ch:设备水印时标 | en:Device frame-specific time scale
    unsigned int        nSecondCount;
    unsigned int        nCycleCount;
    unsigned int        nCycleOffset;

    float               fGain;
    float               fExposureTime;
    unsigned int        nAverageBrightness;     // ch:平均亮度 | en:Average brightness

    // ch:白平衡相关 | en:White balance
    unsigned int        nRed;
    unsigned int        nGreen;
    unsigned int        nBlue;

    unsigned int        nFrameCounter;
    unsigned int        nTriggerIndex;      // ch:触发计数 | en:Trigger Counting

    // ch:Line 输入/输出 | en:Line Input/Output
    unsigned int        nInput;        // ch:输入 | en:Input
    unsigned int        nOutput;       // ch:输出 | en:Output

    // ch:ROI区域 | en:ROI Region
    unsigned short      nOffsetX;
    unsigned short      nOffsetY;
    unsigned short      nChunkWidth;
    unsigned short      nChunkHeight;

    unsigned int        nLostPacket;  // ch:本帧丢包数 | en:Lost Pacekt Number In This Frame

    unsigned int nUnparsedChunkNum;
    union
    {
        MV_CHUNK_DATA_CONTENT* pUnparsedChunkContent;
        int64_t nRet;
    }UnparsedChunkList;

    unsigned int        nReserved[36];       // 保留
}MV_FRAME_OUT_INFO_EX;

// ch:图像结构体，输出图像指针地址及图像信息 | en:Image Struct, output the pointer of Image and the information of the specific image
typedef struct _MV_FRAME_OUT_
{
    unsigned char* pBufAddr;              // ch:图像指针地址 | en: pointer of image
    MV_FRAME_OUT_INFO_EX stFrameInfo;     // ch:图像信息 | en:information of the specific image

    unsigned int nRes[16];                // ch:保留 | en:reserved
}MV_FRAME_OUT;

typedef struct _MV_DISPLAY_FRAME_INFO_
{
    void*                    hWnd;               // ch:窗口句柄 | en:HWND
    unsigned char*           pData;              // ch:显示的数据 | en:Data Buffer
    unsigned int             nDataLen;           // ch:数据长度 | en:Data Size
    unsigned short           nWidth;             // ch:图像宽 | en:Width
    unsigned short           nHeight;            // ch:图像高 | en:Height
    enum MvGvspPixelType     enPixelType;        // ch:像素格式 | en:Pixel format
    unsigned int             nRes[4];

}MV_DISPLAY_FRAME_INFO;

// ch:保存图片格式 | en:Save image type
enum MV_SAVE_IAMGE_TYPE
{
    MV_Image_Undefined                 = 0,
    MV_Image_Bmp                       = 1,
    MV_Image_Jpeg                      = 2,
    MV_Image_Png                       = 3,     // ch:不支持 | en:Not support
    MV_Image_Tif                       = 4,     // ch:不支持 | en:Not support
};
// ch:保存图片参数 | en:Save image type
typedef struct _MV_SAVE_IMAGE_PARAM_T_
{
    unsigned char*        pData;              // [IN]     ch:输入数据缓存 | en:Input Data Buffer
    unsigned int          nDataLen;           // [IN]    ch:输入数据大小 | en:Input Data Size
    enum MvGvspPixelType       enPixelType;        // [IN]     ch:输入数据的像素格式 | en:Input Data Pixel Format
    unsigned short        nWidth;             // [IN]     ch:图像宽 | en:Image Width
    unsigned short        nHeight;            // [IN]     ch:图像高 | en:Image Height

    unsigned char*        pImageBuffer;       // [OUT]    ch:输出图片缓存 | en:Output Image Buffer
    unsigned int          nImageLen;          // [OUT]    ch:输出图片大小 | en:Output Image Size
    unsigned int          nBufferSize;        // [IN]     ch:提供的输出缓冲区大小 | en:Output buffer size provided
    enum MV_SAVE_IAMGE_TYPE    enImageType;        // [IN]     ch:输出图片格式 | en:Output Image Format

}MV_SAVE_IMAGE_PARAM;

// ch:图片保存参数 | en:Save Image Parameters
typedef struct _MV_SAVE_IMAGE_PARAM_T_EX_
{
    unsigned char*      pData;              // [IN]     ch:输入数据缓存 | en:Input Data Buffer
    unsigned int        nDataLen;           // [IN]     ch:输入数据大小 | en:Input Data Size
    enum MvGvspPixelType     enPixelType;        // [IN]     ch:输入数据的像素格式 | en:Input Data Pixel Format
    unsigned short      nWidth;             // [IN]     ch:图像宽 | en:Image Width
    unsigned short      nHeight;            // [IN]     ch:图像高 | en:Image Height

    unsigned char*      pImageBuffer;       // [OUT]    ch:输出图片缓存 | en:Output Image Buffer
    unsigned int        nImageLen;          // [OUT]    ch:输出图片大小 | en:Output Image Size
    unsigned int        nBufferSize;        // [IN]     ch:提供的输出缓冲区大小 | en:Output buffer size provided
    enum MV_SAVE_IAMGE_TYPE  enImageType;        // [IN]     ch:输出图片格式 | en:Output Image Format
    unsigned int        nJpgQuality;        // [IN]     ch:编码质量, (50-99] | en:Encoding quality, (50-99]

    // [IN]     ch:Bayer格式转为RGB24的插值方法  0-最近邻 1-双线性 2-Hamilton （如果传入其它值则默认为最近邻）
	// [IN]     en:Interpolation method of convert Bayer to RGB24  0-nearest neighbour 1-bilinearity 2-Hamilton
    unsigned int        iMethodValue;
    unsigned int        nReserved[3];

}MV_SAVE_IMAGE_PARAM_EX;

// ch:图像转换结构体 | en:Pixel convert structure
typedef struct _MV_PIXEL_CONVERT_PARAM_T_
{
    unsigned short      nWidth;             // [IN]     ch:图像宽 | en:Width
    unsigned short      nHeight;            // [IN]     ch:图像高 | en:Height

    enum MvGvspPixelType    enSrcPixelType;     // [IN]     ch:源像素格式 | en:Source pixel format
    unsigned char*      pSrcData;           // [IN]     ch:输入数据缓存 | en:Input data buffer
    unsigned int        nSrcDataLen;        // [IN]     ch:输入数据大小 | en:Input data size

    enum MvGvspPixelType    enDstPixelType;     // [IN]     ch:目标像素格式 | en:Destination pixel format
    unsigned char*      pDstBuffer;         // [OUT]    ch:输出数据缓存 | en:Output data buffer
    unsigned int        nDstLen;            // [OUT]    ch:输出数据大小 | en:Output data size
    unsigned int        nDstBufferSize;     // [IN]     ch:提供的输出缓冲区大小 | en:Provided outbut buffer size
    unsigned int        nRes[4];
}MV_CC_PIXEL_CONVERT_PARAM;

// ch:录像格式定义 | en:Record Format Type
typedef enum _MV_RECORD_FORMAT_TYPE_
{
    MV_FormatType_Undefined     = 0,
    MV_FormatType_AVI           = 1,

}MV_RECORD_FORMAT_TYPE;

// ch:录像参数 | en:Record Parameters
typedef struct _MV_CC_RECORD_PARAM_T_
{
    enum MvGvspPixelType    enPixelType;    // [IN]     输入数据的像素格式

    unsigned short      nWidth;             // [IN]     图像宽(指定目标参数时需为2的倍数)
    unsigned short      nHeight;            // [IN]     图像高(指定目标参数时需为2的倍数)

    float               fFrameRate;         // [IN]     帧率fps(1/16-120)
    unsigned int        nBitRate;           // [IN]     码率kbps(128kbps-16Mbps)

    MV_RECORD_FORMAT_TYPE enRecordFmtType;  // [IN]     录像格式

    char*               strFilePath;        // [IN]     录像文件存放路径(如果路径中存在中文，需转成utf-8)

    unsigned int        nRes[8];

}MV_CC_RECORD_PARAM;

// ch:录像数据 | en:Record Data
typedef struct _MV_CC_INPUT_FRAME_INFO_T_
{
    unsigned char*      pData;              // [IN]     图像数据指针
    unsigned int        nDataLen;           // [IN]     图像大小

    unsigned int        nRes[8];

}MV_CC_INPUT_FRAME_INFO;

// ch:采集模式 | en:Acquisition mode
typedef enum _MV_CAM_ACQUISITION_MODE_
{
    MV_ACQ_MODE_SINGLE      = 0,            // ch:单帧模式 | en:Single Mode
    MV_ACQ_MODE_MUTLI       = 1,            // ch:多帧模式 | en:Multi Mode
    MV_ACQ_MODE_CONTINUOUS  = 2,            // ch:持续采集模式 | en:Continuous Mode

}MV_CAM_ACQUISITION_MODE;

// ch:增益模式 | en:Gain Mode
typedef enum _MV_CAM_GAIN_MODE_
{
    MV_GAIN_MODE_OFF        = 0,            // ch:关闭 | en:Single Mode
    MV_GAIN_MODE_ONCE       = 1,            // ch:一次 | en:Multi Mode
    MV_GAIN_MODE_CONTINUOUS = 2,            // ch:连续 | en:Continuous Mode

}MV_CAM_GAIN_MODE;

// ch:曝光模式 | en:Exposure Mode
typedef enum _MV_CAM_EXPOSURE_MODE_
{
    MV_EXPOSURE_MODE_TIMED          = 0,            // Timed
    MV_EXPOSURE_MODE_TRIGGER_WIDTH  = 1,            // TriggerWidth
}MV_CAM_EXPOSURE_MODE;

// ch:自动曝光模式 | en:Auto Exposure Mode
typedef enum _MV_CAM_EXPOSURE_AUTO_MODE_
{
    MV_EXPOSURE_AUTO_MODE_OFF        = 0,            // ch:关闭 | en:Off
    MV_EXPOSURE_AUTO_MODE_ONCE       = 1,            // ch:一次 | en:Once
    MV_EXPOSURE_AUTO_MODE_CONTINUOUS = 2,            // ch:连续 | en:Continuous

}MV_CAM_EXPOSURE_AUTO_MODE;

typedef enum _MV_CAM_TRIGGER_MODE_
{
    MV_TRIGGER_MODE_OFF         = 0,            // ch:关闭 | en:Off
    MV_TRIGGER_MODE_ON          = 1,            // ch:打开 | en:ON

}MV_CAM_TRIGGER_MODE;

typedef enum _MV_CAM_GAMMA_SELECTOR_
{
    MV_GAMMA_SELECTOR_USER      = 1,
    MV_GAMMA_SELECTOR_SRGB      = 2,

}MV_CAM_GAMMA_SELECTOR;

typedef enum _MV_CAM_BALANCEWHITE_AUTO_
{
    MV_BALANCEWHITE_AUTO_OFF            = 0,
    MV_BALANCEWHITE_AUTO_ONCE           = 2,
    MV_BALANCEWHITE_AUTO_CONTINUOUS     = 1,            // ch:连续 | en:Continuous

}MV_CAM_BALANCEWHITE_AUTO;

typedef enum _MV_CAM_TRIGGER_SOURCE_
{
    MV_TRIGGER_SOURCE_LINE0             = 0,
    MV_TRIGGER_SOURCE_LINE1             = 1,
    MV_TRIGGER_SOURCE_LINE2             = 2,
    MV_TRIGGER_SOURCE_LINE3             = 3,
    MV_TRIGGER_SOURCE_COUNTER0          = 4,

    MV_TRIGGER_SOURCE_SOFTWARE          = 7,
    MV_TRIGGER_SOURCE_FrequencyConverter= 8,

}MV_CAM_TRIGGER_SOURCE;

typedef enum _MV_GIGE_TRANSMISSION_TYPE_
{
    MV_GIGE_TRANSTYPE_UNICAST                = 0x0,                // ch:表示单播(默认) | en:Unicast mode
    MV_GIGE_TRANSTYPE_MULTICAST              = 0x1,                // ch:表示组播 | en:Multicast mode
    MV_GIGE_TRANSTYPE_LIMITEDBROADCAST       = 0x2,                // ch:表示局域网内广播，暂不支持 | en:Limited broadcast mode,not support
    MV_GIGE_TRANSTYPE_SUBNETBROADCAST        = 0x3,                // ch:表示子网内广播，暂不支持 | en:Subnet broadcast mode,not support
    MV_GIGE_TRANSTYPE_CAMERADEFINED          = 0x4,                // ch:表示从相机获取，暂不支持 | en:Transtype from camera,not support
    MV_GIGE_TRANSTYPE_UNICAST_DEFINED_PORT   = 0x5,                // ch:表示用户自定义应用端接收图像数据Port号 | en:User Defined Receive Data Port
    MV_GIGE_TRANSTYPE_UNICAST_WITHOUT_RECV   = 0x00010000,         // ch:表示设置了单播，但本实例不接收图像数据 | en:Unicast without receive data
    MV_GIGE_TRANSTYPE_MULTICAST_WITHOUT_RECV = 0x00010001,         // ch:表示组播模式，但本实例不接收图像数据 | en:Multicast without receive data
}MV_GIGE_TRANSMISSION_TYPE;

// GigEVision IP Configuration
#define MV_IP_CFG_STATIC        0x05000000
#define MV_IP_CFG_DHCP          0x06000000
#define MV_IP_CFG_LLA           0x04000000

// GigEVision Net Transfer Mode
#define MV_NET_TRANS_DRIVER     0x00000001
#define MV_NET_TRANS_SOCKET     0x00000002

// CameraLink Baud Rates (CLUINT32)
#define MV_CAML_BAUDRATE_9600             0x00000001
#define MV_CAML_BAUDRATE_19200           0x00000002
#define MV_CAML_BAUDRATE_38400           0x00000004
#define MV_CAML_BAUDRATE_57600           0x00000008
#define MV_CAML_BAUDRATE_115200          0x00000010
#define MV_CAML_BAUDRATE_230400          0x00000020
#define MV_CAML_BAUDRATE_460800          0x00000040
#define MV_CAML_BAUDRATE_921600          0x00000080
#define MV_CAML_BAUDRATE_AUTOMAX         0x40000000


// ch:信息类型 | en:Information Type
#define MV_MATCH_TYPE_NET_DETECT             0x00000001      // ch:网络流量和丢包信息 | en:Network traffic and packet loss information
#define MV_MATCH_TYPE_USB_DETECT             0x00000002      // ch:host接收到来自U3V设备的字节总数 | en:The total number of bytes host received from U3V device


// ch:某个节点对应的子节点个数最大值 | en:The maximum number of child nodes corresponding to a node
#define MV_MAX_XML_NODE_NUM_C       128

// ch:节点名称字符串最大长度 | en:The maximum length of node name string
#define MV_MAX_XML_NODE_STRLEN_C    64

// ch:节点String值最大长度 | en:The maximum length of Node String
#define MV_MAX_XML_STRVALUE_STRLEN_C 64

// ch:节点描述字段最大长度 | en:The maximum length of the node description field
#define MV_MAX_XML_DISC_STRLEN_C    512

// ch:最多的单元数 | en:The maximum number of units
#define MV_MAX_XML_ENTRY_NUM        10

// ch:父节点个数上限 | en:The maximum number of parent nodes
#define MV_MAX_XML_PARENTS_NUM      8

// ch:每个已经实现单元的名称长度 | en:The length of the name of each unit that has been implemented
#define MV_MAX_XML_SYMBOLIC_STRLEN_C 64

#define MV_MAX_XML_SYMBOLIC_NUM      64

// ch:重发包默认最大包数量 | en:The default maximum number of retransmission packets

// ch:全匹配的一种信息结构体 | en:A fully matched information structure
typedef struct _MV_ALL_MATCH_INFO_
{
    unsigned int    nType;          // ch:需要输出的信息类型，e.g. MV_MATCH_TYPE_NET_DETECT | en:Information type need to output ,e.g. MV_MATCH_TYPE_NET_DETECT
    void*           pInfo;          // ch:输出的信息缓存，由调用者分配 | en:Output information cache, which is allocated by the caller
    unsigned int    nInfoSize;      // ch:信息缓存的大小 | en:Information cache size

}MV_ALL_MATCH_INFO;

// ch:网络流量和丢包信息反馈结构体，对应类型为 MV_MATCH_TYPE_NET_DETECT
// en:Network traffic and packet loss feedback structure, the corresponding type is MV_MATCH_TYPE_NET_DETECT
typedef struct _MV_MATCH_INFO_NET_DETECT_
{
    int64_t         nReviceDataSize;    // ch:已接收数据大小  [统计StartGrabbing和StopGrabbing之间的数据量] | en:Received data size 
    int64_t         nLostPacketCount;   // ch:丢失的包数量 | en:Number of packets lost
    unsigned int    nLostFrameCount;    // ch:丢帧数量 | en:Number of frames lost
    unsigned int    nNetRecvFrameCount;          // ch:保留 | en:Reserved
    int64_t         nRequestResendPacketCount;          // 请求重发包数
    int64_t         nResendPacketCount;   // 重发包数
}MV_MATCH_INFO_NET_DETECT;

// ch:host收到从u3v设备端的总字节数，对应类型为 MV_MATCH_TYPE_USB_DETECT | en:The total number of bytes host received from the u3v device side, the corresponding type is MV_MATCH_TYPE_USB_DETECT
typedef struct _MV_MATCH_INFO_USB_DETECT_
{
    int64_t         nReviceDataSize;      // ch:已接收数据大小    [统计OpenDevicce和CloseDevice之间的数据量] | en:Received data size
    unsigned int    nRevicedFrameCount;   // ch:已收到的帧数 | en:Number of frames received
    unsigned int    nErrorFrameCount;     // ch:错误帧数 | en:Number of error frames
    unsigned int    nReserved[2];         // ch:保留 | en:Reserved
}MV_MATCH_INFO_USB_DETECT;

typedef struct _MV_IMAGE_BASIC_INFO_
{
    // width
    unsigned short      nWidthValue;
    unsigned short      nWidthMin;
    unsigned int        nWidthMax;
    unsigned int        nWidthInc;

    // height
    unsigned int        nHeightValue;
    unsigned int        nHeightMin;
    unsigned int        nHeightMax;
    unsigned int        nHeightInc;

    // framerate
    float               fFrameRateValue;
    float               fFrameRateMin;
    float               fFrameRateMax;

    // ch:像素格式 | en:pixel format
    unsigned int        enPixelType;                // ch:当前的像素格式 | en:Current pixel format
    unsigned int        nSupportedPixelFmtNum;      // ch:支持的像素格式种类 | en:Support pixel format
    unsigned int        enPixelList[MV_MAX_XML_SYMBOLIC_NUM];
    unsigned int        nReserved[8];

}MV_IMAGE_BASIC_INFO;

// ch: 异常消息类型 | en:Exception message type
#define MV_EXCEPTION_DEV_DISCONNECT     0x00008001      // ch:设备断开连接 | en:The device is disconnected
#define MV_EXCEPTION_VERSION_CHECK      0x00008002      // ch:SDK与驱动版本不匹配 | en:SDK does not match the driver version


// ch:设备的访问模式 | en:Device Access Mode
// ch:独占权限，其他APP只允许读CCP寄存器 | en:Exclusive authority, other APP is only allowed to read the CCP register
#define MV_ACCESS_Exclusive                                         1
// ch:可以从5模式下抢占权限，然后以独占权限打开 | en:You can seize the authority from the 5 mode, and then open with exclusive authority
#define MV_ACCESS_ExclusiveWithSwitch                               2
// ch:控制权限，其他APP允许读所有寄存器 | en:Control authority, allows other APP reading all registers
#define MV_ACCESS_Control                                           3
// ch:可以从5的模式下抢占权限，然后以控制权限打开 | en:You can seize the authority from the 5 mode, and then open with control authority
#define MV_ACCESS_ControlWithSwitch                                 4
// ch:以可被抢占的控制权限打开 | en:Open with seized control authority
#define MV_ACCESS_ControlSwitchEnable                               5
// ch:可以从5的模式下抢占权限，然后以可被抢占的控制权限打开 | en:You can seize the authority from the 5 mode, and then open with seized control authority
#define MV_ACCESS_ControlSwitchEnableWithKey                        6
// ch:读模式打开设备，适用于控制权限下 | en:Open with read mode and is available under control authority
#define MV_ACCESS_Monitor                                           7


/************************************************************************/
/* 封装了GenICam的C接口相关参数定义                                     */
/* Package of GenICam C interface-related parameters definition         */
/************************************************************************/

// ch:每个节点对应的接口类型 | en:Interface type corresponds to each node 
enum MV_XML_InterfaceType
{
    IFT_IValue,         //!> IValue interface
    IFT_IBase,          //!> IBase interface
    IFT_IInteger,       //!> IInteger interface
    IFT_IBoolean,       //!> IBoolean interface
    IFT_ICommand,       //!> ICommand interface
    IFT_IFloat,         //!> IFloat interface
    IFT_IString,        //!> IString interface
    IFT_IRegister,      //!> IRegister interface
    IFT_ICategory,      //!> ICategory interface
    IFT_IEnumeration,   //!> IEnumeration interface
    IFT_IEnumEntry,     //!> IEnumEntry interface
    IFT_IPort,          //!> IPort interface
};

// ch:节点的访问模式 | en:Node Access Mode
enum MV_XML_AccessMode
{
    AM_NI,          //!< Not implemented
    AM_NA,          //!< Not available
    AM_WO,          //!< Write Only
    AM_RO,          //!< Read Only
    AM_RW,          //!< Read and Write
    AM_Undefined,   //!< Object is not yet initialized
    AM_CycleDetect, //!< used internally for AccessMode cycle detection

};

enum MV_XML_Visibility
{
    V_Beginner      = 0,    //!< Always visible
    V_Expert        = 1,    //!< Visible for experts or Gurus
    V_Guru          = 2,    //!< Visible for Gurus
    V_Invisible     = 3,    //!< Not Visible
    V_Undefined     = 99    //!< Object is not yet initialized
};

//Event事件回调信息 | en:Event callback infomation
#define MAX_EVENT_NAME_SIZE     128//相机Event事件名称最大长度 | en:Max length of event name

typedef struct _MV_EVENT_OUT_INFO_
{
    char            EventName[MAX_EVENT_NAME_SIZE];     //Event名称 | en:Event name

    unsigned short  nEventID;                           //Event号 | en:Event ID
    unsigned short  nStreamChannel;                     //流通道序号 | en:Circulation number

    unsigned int    nBlockIdHigh;                       //帧号高位 | en:BlockId high
    unsigned int    nBlockIdLow;                        //帧号低位 | en:BlockId low

    unsigned int    nTimestampHigh;                     //时间戳高位 | en:Timestramp high
    unsigned int    nTimestampLow;                      //时间戳低位 | en:Timestramp low

    void *          pEventData;                         //Event数据 | en:Event data
    unsigned int    nEventDataSize;                     //Event数据长度 | en:Event data len

    unsigned int    nReserved[16];                      //预留 | en:Reserved
}MV_EVENT_OUT_INFO;

// ch:文件存取 | en:File Access
typedef struct _MV_CC_FILE_ACCESS_T
{
    const char * pUserFileName;                         //用户文件名 | en:User file name
    const char * pDevFileName;                          //设备文件名 | en:Device file name

    unsigned int    nReserved[32];                      //预留 | en:Reserved
}MV_CC_FILE_ACCESS;

// ch:文件存取进度 | en:File Access Progress
typedef struct _MV_CC_FILE_ACCESS_PROGRESS_T
{
    int64_t nCompleted;                               //已完成的长度 | en:Completed Length
    int64_t nTotal;                                   //总长度 | en:Total Length

    unsigned int    nReserved[8];                      //预留 | en:Reserved
}MV_CC_FILE_ACCESS_PROGRESS;

// ch:传输模式，可以为单播模式、组播模式等 | en:Transmission type
typedef struct _MV_TRANSMISSION_TYPE_T
{
    MV_GIGE_TRANSMISSION_TYPE  enTransmissionType; //传输模式 | en:Transmission type
    unsigned int                    nDestIp;            //目标IP，组播模式下有意义 | en:Destination IP
    unsigned short                  nDestPort;          //目标Port，组播模式下有意义 | en:Destination port

    unsigned int    nReserved[32];                      //预留 | en:Reserved
}MV_TRANSMISSION_TYPE;

// ch:动作命令信息 | en:Action Command
typedef struct _MV_ACTION_CMD_INFO_T
{
    unsigned int        nDeviceKey;        //设备密钥
    unsigned int        nGroupKey;         //组键
    unsigned int        nGroupMask;        //组掩码

    unsigned int        bActionTimeEnable; //只有设置成1时Action Time才有效，非1时无效
    int64_t             nActionTime;       //预定的时间，和主频有关

    const char*         pBroadcastAddress; //广播包地址
    unsigned int        nTimeOut;          //等待ACK的超时时间，如果为0表示不需要ACK

    unsigned int        nReserved[16];     //预留 | en:Reserved

}MV_ACTION_CMD_INFO;

// ch:动作命令返回信息 | en:Action Command Result
typedef struct _MV_ACTION_CMD_RESULT_T
{
    unsigned char       strDeviceAddress[12 + 3 + 1];  //IP address of the device

    //status code returned by the device
    int                 nStatus;//1.0x0000:success.
                                //2.0x8001:Command is not supported by the device.
                                //3.0x8013:The device is not synchronized to a master clock to be used as time reference.
                                //4.0x8015:A device queue or packet data has overflowed.
                                //5.0x8016:The requested scheduled action command was requested at a time that is already past.

    unsigned int        nReserved[4];         //预留 | en:Reserved

}MV_ACTION_CMD_RESULT;

// ch:动作命令返回信息列表 | en:Action Command Result List
typedef struct _MV_ACTION_CMD_RESULT_LIST_T
{
    unsigned int            nNumResults;      //返回值个数
    MV_ACTION_CMD_RESULT*   pResults;

}MV_ACTION_CMD_RESULT_LIST;

// ch:单个节点基本属性 | en:Single Node Basic Attributes
typedef struct _MV_XML_NODE_FEATURE_
{
    enum MV_XML_InterfaceType    enType;                             // ch:节点类型 | en:Node Type
    enum MV_XML_Visibility       enVisivility;                       // ch:是否可见 | en:Is visibility
    char                    strDescription[MV_MAX_XML_DISC_STRLEN_C];// ch:节点描述     目前暂不支持 | en:Node Description, NOT SUPPORT NOW
    char                    strDisplayName[MV_MAX_XML_NODE_STRLEN_C];// ch:显示名称 | en:Display Name
    char                    strName[MV_MAX_XML_NODE_STRLEN_C];  // ch:节点名 | en:Node Name
    char                    strToolTip[MV_MAX_XML_DISC_STRLEN_C];  // ch:提示 | en:Notice

    unsigned int            nReserved[4];
}MV_XML_NODE_FEATURE;

// ch:节点列表 | en:Node List
typedef struct _MV_XML_NODES_LIST_
{
    unsigned int            nNodeNum;       // ch:节点个数 | en:Node Number
    MV_XML_NODE_FEATURE     stNodes[MV_MAX_XML_NODE_NUM_C];
}MV_XML_NODES_LIST;



typedef struct _MV_XML_FEATURE_Value_
{
    enum MV_XML_InterfaceType    enType;                             // ch:节点类型 | en:Node Type
    char                    strDescription[MV_MAX_XML_DISC_STRLEN_C];// ch:节点描述     目前暂不支持 | en:Node Description, NOT SUPPORT NOW
    char                    strDisplayName[MV_MAX_XML_NODE_STRLEN_C];// ch:显示名称 | en:Display Name
    char                    strName[MV_MAX_XML_NODE_STRLEN_C];  // ch:节点名 | en:Node Name
    char                    strToolTip[MV_MAX_XML_DISC_STRLEN_C];  // ch:提示 | en:Notice
    unsigned int            nReserved[4];
}MV_XML_FEATURE_Value;

typedef struct _MV_XML_FEATURE_Base_
{
    enum MV_XML_AccessMode   enAccessMode;       // ch:访问模式 | en:Access Mode
}MV_XML_FEATURE_Base;

typedef struct _MV_XML_FEATURE_Integer_
{
    char                strName[MV_MAX_XML_NODE_STRLEN_C];
    char                strDisplayName[MV_MAX_XML_NODE_STRLEN_C];
    char                strDescription[MV_MAX_XML_DISC_STRLEN_C];   // ch:目前暂不支持 | en:NOT SUPPORT NOW
    char                strToolTip[MV_MAX_XML_DISC_STRLEN_C];

    enum MV_XML_Visibility   enVisivility;                       //是否可见 | en:Visible
    enum MV_XML_AccessMode   enAccessMode;       // ch:访问模式 | en:Access Mode
    int                 bIsLocked;          // ch:是否锁定。0-否；1-是    目前暂不支持 | en:Locked. 0-NO; 1-YES, NOT SUPPORT NOW
    int64_t             nValue;             // ch:当前值 | en:Current Value
    int64_t             nMinValue;          // ch:最小值 | en:Min Value
    int64_t             nMaxValue;          // ch:最大值 | en:Max Value
    int64_t             nIncrement;         // ch:增量 | en:Increment

    unsigned int        nReserved[4];

}MV_XML_FEATURE_Integer;

typedef struct _MV_XML_FEATURE_Boolean_
{
    char                strName[MV_MAX_XML_NODE_STRLEN_C];
    char                strDisplayName[MV_MAX_XML_NODE_STRLEN_C];
    char                strDescription[MV_MAX_XML_DISC_STRLEN_C];   // ch:目前暂不支持 | en:NOT SUPPORT NOW
    char                strToolTip[MV_MAX_XML_DISC_STRLEN_C];

    enum MV_XML_Visibility   enVisivility;                       // ch:是否可见 | en:Visible
    enum MV_XML_AccessMode   enAccessMode;       // ch:访问模式 | en:Access Mode
    int                 bIsLocked;          // ch:是否锁定。0-否；1-是    目前暂不支持 | en:Locked. 0-NO; 1-YES, NOT SUPPORT NOW
    bool                bValue;             // ch:当前值 | en:Current Value

    unsigned int        nReserved[4];
}MV_XML_FEATURE_Boolean;

typedef struct _MV_XML_FEATURE_Command_
{
    char                strName[MV_MAX_XML_NODE_STRLEN_C];
    char                strDisplayName[MV_MAX_XML_NODE_STRLEN_C];
    char                strDescription[MV_MAX_XML_DISC_STRLEN_C];   // ch:目前暂不支持 | en:NOT SUPPORT NOW
    char                strToolTip[MV_MAX_XML_DISC_STRLEN_C];

    enum MV_XML_Visibility   enVisivility;                       // ch:是否可见 | en:Visible
    enum MV_XML_AccessMode   enAccessMode;       // ch:访问模式 | en:Access Mode
    int                 bIsLocked;          // ch:是否锁定。0-否；1-是    目前暂不支持 | en:Locked. 0-NO; 1-YES, NOT SUPPORT NOW

    unsigned int        nReserved[4];
}MV_XML_FEATURE_Command;

typedef struct _MV_XML_FEATURE_Float_
{
    char                strName[MV_MAX_XML_NODE_STRLEN_C];
    char                strDisplayName[MV_MAX_XML_NODE_STRLEN_C];
    char                strDescription[MV_MAX_XML_DISC_STRLEN_C];   // ch:目前暂不支持 | en:NOT SUPPORT NOW
    char                strToolTip[MV_MAX_XML_DISC_STRLEN_C];

    enum MV_XML_Visibility       enVisivility;                       //是否可见 | en:Visible
    enum MV_XML_AccessMode   enAccessMode;       // ch:访问模式 | en:Access Mode
    int                 bIsLocked;          // ch:是否锁定。0-否；1-是    目前暂不支持 | en:Locked. 0-NO; 1-YES, NOT SUPPORT NOW
    double              dfValue;             // ch:当前值 | en:Current Value
    double              dfMinValue;          // ch:最小值 | en:Min Value
    double              dfMaxValue;          // ch:最大值 | en:Max Value
    double              dfIncrement;         // ch:增量 | en:Increment

    unsigned int        nReserved[4];
}MV_XML_FEATURE_Float;

typedef struct _MV_XML_FEATURE_String_
{
    char                strName[MV_MAX_XML_NODE_STRLEN_C];
    char                strDisplayName[MV_MAX_XML_NODE_STRLEN_C];
    char                strDescription[MV_MAX_XML_DISC_STRLEN_C];   // ch:目前暂不支持 | en:NOT SUPPORT NOW
    char                strToolTip[MV_MAX_XML_DISC_STRLEN_C];

    enum MV_XML_Visibility       enVisivility;                       // ch:是否可见 | en:Visible
    enum MV_XML_AccessMode   enAccessMode;       // ch:访问模式 | en:Access Mode
    int                 bIsLocked;          // ch:是否锁定。0-否；1-是    目前暂不支持 | en:Locked. 0-NO; 1-YES, NOT SUPPORT NOW
    char                strValue[MV_MAX_XML_STRVALUE_STRLEN_C];// ch:当前值 | en:Current Value

    unsigned int        nReserved[4];
}MV_XML_FEATURE_String;

typedef struct _MV_XML_FEATURE_Register_
{
    char                strName[MV_MAX_XML_NODE_STRLEN_C];
    char                strDisplayName[MV_MAX_XML_NODE_STRLEN_C];
    char                strDescription[MV_MAX_XML_DISC_STRLEN_C];   // ch:目前暂不支持 | en:NOT SUPPORT NOW
    char                strToolTip[MV_MAX_XML_DISC_STRLEN_C];

    enum MV_XML_Visibility       enVisivility;                       // ch:是否可见 | en:Visible
    enum MV_XML_AccessMode   enAccessMode;       // ch:访问模式 | en:Access Mode
    int                 bIsLocked;          // ch:是否锁定。0-否；1-是    目前暂不支持 | en:Locked. 0-NO; 1-YES, NOT SUPPORT NOW
    int64_t             nAddrValue;             // ch:当前值 | en:Current Value

    unsigned int        nReserved[4];
}MV_XML_FEATURE_Register;

typedef struct _MV_XML_FEATURE_Category_
{
    char                    strDescription[MV_MAX_XML_DISC_STRLEN_C];// ch:节点描述 目前暂不支持 | en:Node Description, NOT SUPPORT NOW
    char                    strDisplayName[MV_MAX_XML_NODE_STRLEN_C];// ch:显示名称 | en:Display Name
    char                    strName[MV_MAX_XML_NODE_STRLEN_C];  // ch:节点名 | en:Node Name
    char                    strToolTip[MV_MAX_XML_DISC_STRLEN_C];  // ch:提示 | en:Notice

    enum MV_XML_Visibility       enVisivility;                       // ch:是否可见 | en:Visible

    unsigned int            nReserved[4];
}MV_XML_FEATURE_Category;

typedef struct _MV_XML_FEATURE_EnumEntry_
{
    char                strName[MV_MAX_XML_NODE_STRLEN_C];
    char                strDisplayName[MV_MAX_XML_NODE_STRLEN_C];
    char                strDescription[MV_MAX_XML_DISC_STRLEN_C];   // ch:目前暂不支持 | en:NOT SUPPORT NOW
    char                strToolTip[MV_MAX_XML_DISC_STRLEN_C];
    int                 bIsImplemented;
    int                 nParentsNum;
    MV_XML_NODE_FEATURE stParentsList[MV_MAX_XML_PARENTS_NUM];

    enum MV_XML_Visibility       enVisivility;                       // ch:是否可见 | en:Visible
    int64_t             nValue;             // ch:当前值 | en:Current Value
    enum MV_XML_AccessMode   enAccessMode;       // ch:访问模式 | en:Access Mode
    int                 bIsLocked;          // ch:是否锁定。0-否；1-是    目前暂不支持 | en:Locked. 0-NO; 1-YES, NOT SUPPORT NOW
    int                 nReserved[8];

}MV_XML_FEATURE_EnumEntry;


typedef struct _MV_XML_FEATURE_Enumeration_
{
    enum MV_XML_Visibility       enVisivility;                       // ch:是否可见 | en:Visible
    char                    strDescription[MV_MAX_XML_DISC_STRLEN_C];// ch:节点描述 目前暂不支持 | en:Node Description, NOT SUPPORT NOW
    char                    strDisplayName[MV_MAX_XML_NODE_STRLEN_C];// ch:显示名称 | en:Display Name
    char                    strName[MV_MAX_XML_NODE_STRLEN_C];  // ch:节点名 | en:Node Name
    char                    strToolTip[MV_MAX_XML_DISC_STRLEN_C];  // ch:提示 | en:Notice

    int                 nSymbolicNum;          // ch:Symbolic数 | en:Symbolic Number
    char                strCurrentSymbolic[MV_MAX_XML_SYMBOLIC_STRLEN_C];          // ch:当前Symbolic索引 | en:Current Symbolic Index
    char                strSymbolic[MV_MAX_XML_SYMBOLIC_NUM][MV_MAX_XML_SYMBOLIC_STRLEN_C];
    enum MV_XML_AccessMode   enAccessMode;       // ch:访问模式 | en:Access Mode
    int                 bIsLocked;          // ch:是否锁定。0-否；1-是    目前暂不支持 | en:Locked. 0-NO; 1-YES, NOT SUPPORT NOW
    int64_t             nValue;             // ch:当前值 | en:Current Value

    unsigned int        nReserved[4];
}MV_XML_FEATURE_Enumeration;


typedef struct _MV_XML_FEATURE_Port_
{
    enum MV_XML_Visibility       enVisivility;                       // ch:是否可见 | en:Visible
    char                    strDescription[MV_MAX_XML_DISC_STRLEN_C];// ch:节点描述 目前暂不支持 | en:Node Description, NOT SUPPORT NOW
    char                    strDisplayName[MV_MAX_XML_NODE_STRLEN_C];// ch:显示名称 | en:Display Name
    char                    strName[MV_MAX_XML_NODE_STRLEN_C];  // ch:节点名 | en:Node Name
    char                    strToolTip[MV_MAX_XML_DISC_STRLEN_C];  // ch:提示 | en:Notice

    enum MV_XML_AccessMode       enAccessMode;       // ch:访问模式 | en:Access Mode
    int                     bIsLocked;          // ch:是否锁定。0-否；1-是    目前暂不支持 | en:Locked. 0-NO; 1-YES, NOT SUPPORT NOW

    unsigned int            nReserved[4];
}MV_XML_FEATURE_Port;



typedef struct _MV_XML_CAMERA_FEATURE_
{
    enum MV_XML_InterfaceType    enType;
    union
    {
        MV_XML_FEATURE_Integer      stIntegerFeature;
        MV_XML_FEATURE_Float        stFloatFeature;
        MV_XML_FEATURE_Enumeration  stEnumerationFeature;
        MV_XML_FEATURE_String       stStringFeature;
    }SpecialFeature;

}MV_XML_CAMERA_FEATURE;


typedef struct _MVCC_ENUMVALUE_T
{
    unsigned int    nCurValue;      // ch:当前值 | en:Current Value
    unsigned int    nSupportedNum;  // ch:数据的有效数据个数 | en:Number of valid data
    unsigned int    nSupportValue[MV_MAX_XML_SYMBOLIC_NUM];

    unsigned int    nReserved[4];
}MVCC_ENUMVALUE;

typedef struct _MVCC_INTVALUE_T
{
    unsigned int    nCurValue;      // ch:当前值 | en:Current Value
    unsigned int    nMax;
    unsigned int    nMin;
    unsigned int    nInc;

    unsigned int    nReserved[4];
}MVCC_INTVALUE;

typedef struct _MVCC_INTVALUE_EX_T
{
    int64_t    nCurValue;      // ch:当前值 | en:Current Value
    int64_t    nMax;
    int64_t    nMin;
    int64_t    nInc;

    unsigned int    nReserved[16];
}MVCC_INTVALUE_EX;

typedef struct _MVCC_FLOATVALUE_T
{
    float           fCurValue;      // ch:当前值 | en:Current Value
    float           fMax;
    float           fMin;

    unsigned int    nReserved[4];
}MVCC_FLOATVALUE;

typedef struct _MVCC_STRINGVALUE_T
{
    char   chCurValue[256];      // ch:当前值 | en:Current Value

    int64_t nMaxLength;
    unsigned int    nReserved[2];
}MVCC_STRINGVALUE;

#endif /* _MV_CAMERA_PARAMS_H_ */
