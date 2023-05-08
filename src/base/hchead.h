#ifndef HCHEAD_H
#define HCHEAD_H

#include <set>
#include <chrono>
#include <algorithm>
#include <vector>
#include <list>
#include <iostream>
#include <cmath>
#include <ctime> 
#include <stdarg.h> 

#include "HcData.h"

#define SDK_VER                    (char*)"V3.2.23d"

#define SHARK_ENABLE               0

#define  DEBUG_INFO                0

#define  DEFAULT_ID                     (char*)"00000000000000000000000000000000"
#define  DEFAULT_FACTORY                (char*)"CS"
#define  DEFAULT_FIRMWARE               (char*)"00.00.00.00"
#define  DEFAULT_HARDWARE               (char*)"00.00.00.00"

#define  X1B                            "X1B"
#define  X1D                            "X1D"
#define  X1E                            "X1E"
#define  X1F                            "X1F"
#define  X1G                            "X1G"
#define  X1K                            "X1K"
#define  X1L                            "X1L"
#define  X1M                            "X1M"
#define  X1N                            "X1N"
#define  X1S                            "X1S"
#define  X2A                            "X2A"
#define  X2B                            "X2B"
#define  X2C                            "X2C"
#define  X2D                            "X2D"
#define  X2E                            "X2E"
#define  X2F                            "X2F"
#define  X2M                            "X2M"
#define  X2N                            "X2N"
#define  X2BF                           "X2BF"//speed 6Hz/3Hz
#define  X2BZ                           "X2BZ"
#define  X2B2                           "X2B2"
#define  X2DE                           "X2DE"
#define  X2MF                           "X2MF"//speed 6Hz/3Hz
#define  X2MX                           "X2MX"
#define  X2MM                           "X2MM"
//#define  X2YJ                           "X2YJ"
//#define  X2YP                           "X2YP"
#define  X2YE                           "X2YE"

#define  D2A                            "D2A"
#define  D2B                            "D2B"
#define  D2M8                           "D2M8"
#define  D2A8                           "D2A8"
#define  D2B8                           "D2B8"
#define  D2S8                           "D2S8"
#define  D2SA                           "D2SA"
#define  D2P8                           "D2P8"
#define  D2PD                           "D2PD"
#define  D2M1                           "D2M1"
#define  D2AE                           "D2AE"
#define  D2M7                           "D2M7"

#define  T3B                            "T3B"

#define  PI_HC                          3.141592653589793

#define  READ_BUFF_SIZE                 256
#define  READ_TIMEOUT_MS                10
#define  DEFAULT_ID_LEN                 14

#define  FACT_NEW_LEN                   6
#define  FACT_NEW_RESERVE_LEN           6
#define  FACT_NEW_CAL_LEN               2
#define  FACT_NEW_HARD_LEN              3
#define  FACT_SPEED_VER_LEN             2
#define  FACT_OTHER_RESERVE_LEN         2
#define  FACT_V06_RESERVE_LEN           3
#define  FACT_SN_LEN                    20

#define  FAC_INFO_LEN                   12
#define  RESERVER_LEN                   4
#define  VER_LEN                        3
#define  ID_LEN                         4
#define  DIST_BYTES                     2

#define  MSG_ID                         1
#define  MSG_CMD                        2
#define  MSG_POINTCLOUD                 3
#define  MSG_NEW_SN                     4

#define  FPS_1800_NOR                   1800
#define  FPS_2000_NOR                   2088 //2088  // 2100
#define  FPS_3000_NOR                   3048 //3012 
#define  FPS_TOF_NOR                    3300
#define  FPS_1800_RANGE                 50
#define  FPS_2000_RANGE                 100
#define  FPS_3000_RANGE                 100
#define  FPS_TOF_RANGE                  200
#define  FPS_1800_MAX                   (FPS_1800_NOR+FPS_1800_RANGE)
#define  FPS_1800_MIN                   (FPS_1800_NOR-FPS_1800_RANGE)
#define  FPS_2000_MAX                   (FPS_2000_NOR+FPS_2000_RANGE)
#define  FPS_2000_MIN                   (FPS_2000_NOR-FPS_2000_RANGE)
#define  FPS_3000_MAX                   (FPS_3000_NOR+FPS_3000_RANGE)
#define  FPS_3000_MIN                   (FPS_3000_NOR-FPS_3000_RANGE)
#define  FPS_TOF_MAX                    (FPS_TOF_NOR+FPS_TOF_RANGE)
#define  FPS_TOF_MIN                    (FPS_TOF_NOR-FPS_TOF_RANGE)

#define  ANGLE_RESOLV_1800              1.09
#define  ANGLE_RESOLV_2000              0.92  //0.92
#define  ANGLE_RESOLV_2000_6HZ          1.05 
#define  ANGLE_RESOLV_3000              0.72 //0.75 6HZ
#define  ANGLE_RESOLV_TOF               0.7
#define  ANGLE_RESOLV_NARWAL_NOR        0.65 //
#define  ANGLE_RESOLV_NARWAL_LOW        0.55 //

#define  ANGLE_RESOLV_2000_3HZ          0.52  
#define  ANGLE_RESOLV_3000_3HZ          0.36
#define  ANGLE_RESOLV_3000_52HZ         0.65 //0.62

#define  CICRLE_MAX_1800                370
#define  CICRLE_MAX_2000                415  // 5.2hz
#define  CICRLE_MAX_2000_6HZ            360  // 6hz
#define  CICRLE_MAX_3000_52HZ           592  // 5.2hz
#define  CICRLE_MAX_3000                515  // 6hz
#define  CICRLE_MAX_TOF                 580  // 6hz
#define  CICRLE_MAX_NARWAL_NOR          585  // 5.3hz  571
#define  CICRLE_MAX_NARWAL_LOW          760  // 4.2hz   727
#define  CICRLE_MAX_2000_3HZ            700  // 3hz
#define  CICRLE_MAX_3000_3HZ            1015  // 3hz

#define  SPEED_180_NOR                  180
#define  SPEED_250_NOR                  250
#define  SPEED_300_NOR                  300
#define  SPEED_312_NOR                  312
#define  SPEED_315_NOR                  315
#define  SPEED_360_NOR                  360
#define  SPEED_TOF_NOR                  360
#define  SPEED_250_RANGE                10
#define  SPEED_300_RANGE                10
#define  SPEED_312_RANGE                6
#define  SPEED_315_RANGE                10
#define  SPEED_360_RANGE                10
#define  SPEED_TOF_RANGE                10
#define  SPEED_180_MAX                  (SPEED_180_NOR+SPEED_250_RANGE)
#define  SPEED_180_MIN                  (SPEED_180_NOR-SPEED_250_RANGE)
#define  SPEED_250_MAX                  (SPEED_250_NOR+SPEED_250_RANGE)
#define  SPEED_250_MIN                  (SPEED_250_NOR-SPEED_250_RANGE)
#define  SPEED_300_MAX                  (SPEED_300_NOR+SPEED_300_RANGE)
#define  SPEED_300_MIN                  (SPEED_300_NOR-SPEED_300_RANGE)
#define  SPEED_312_MAX                  (SPEED_312_NOR+SPEED_312_RANGE)
#define  SPEED_312_MIN                  (SPEED_312_NOR-SPEED_312_RANGE)
#define  SPEED_315_MAX                  (SPEED_315_NOR+SPEED_315_RANGE)
#define  SPEED_315_MIN                  (SPEED_315_NOR-SPEED_315_RANGE)
#define  SPEED_360_MAX                  (SPEED_360_NOR+SPEED_360_RANGE)
#define  SPEED_360_MIN                  (SPEED_360_NOR-SPEED_360_RANGE)
#define  SPEED_TOF_MAX                  (SPEED_TOF_NOR+SPEED_TOF_RANGE)
#define  SPEED_TOF_MIN                  (SPEED_TOF_NOR-SPEED_TOF_RANGE)

#define  MCU_BLOCK_TIME_MS              1500
#define  INIT_TIMEOUT_MS                2000
#define  LESS_THAN_NUMBER               32  
#define  VALID_NUMBER_COUNT             50 
#define  NUMBER_CONTINUE_CIRCLE         50  
#define  NUMBER_CONTINUE_ERROR_PACKET   10  

#define  SENSOR_ERROR_SECOND            5  
#define  SENSOR_ERROR_TIME_MS           1000  
#define  ENCODER_ERROR_SECOND           5  
#define  ENCODER_ERROR_TIME_MS          1000  
#define  LDS_VOLTAGE_ERROR_SECOND       5  
#define  PD_ERROR_TIME_MS               3000  

#define  PC_GRAY_IS_LATTICE             255  

union Fp32
{
	uint32_t u;
	float f;
};



#pragma pack(push)
#pragma pack(1)


////rock  konyun 2021-04-28
typedef struct tsUID 
{
	UINT16 UID_0;
	UINT16 UID_1;
	UINT16 UID_2;
	UINT16 UID_3;
	UINT16 UID_4;
	UINT16 UID_5;
	UINT16 UID_6;
	UINT16 UID_7;
	UINT16 UID_8;
}tsUID;

typedef struct tsMCUType 
{
	UINT16 word_0;
	UINT16 word_1;
	UINT16 word_2;
	UINT16 word_3;
	UINT16 word_4;
	UINT16 word_5;
}tsMCUType;

typedef struct tsLidarAttr
{
	UINT16    u16PacketSize;  //size 2
	UINT16    u16Version; //size 2
	UINT16    u16UIDSize; //size 2
	tsUID     sUid; //size 18
	UINT16    u16AngleOffset; //size 2
	tsMCUType sMCUType; //size 12
	UINT16    u16LightPlane; //size 2
	UINT16    u16Power; //size 2
	UINT16    u16Calibration[20]; //size 40
	UINT16    u16CheckSum; //size 2
}tsLidarAttr;

typedef union unLidarInfo
{
	tsLidarAttr sAttr;
	UINT8       u8DataOctal[84];
	UINT16      u16DataHex[42];
}unLidarInfo;

#define RCV_SIZE             sizeof(unLidarInfo)*2
#define LDS_INFO_START       0xAA


typedef union unDevID
{
    UINT32       u32ID;
    UCHAR        u8ID[ID_LEN];
}unDevID;


typedef struct tsSDKIDNew
{
    UINT16       u16Head;
    UINT16       u16Len;
    UINT16       u16Cal;
    UCHAR        u8Type;
    UCHAR        u8FacInfo[FACT_NEW_LEN];
    UCHAR        u8FacReserve[FACT_NEW_RESERVE_LEN];
    UINT16       u16Ang;
    UCHAR        u8Direction;
    UCHAR        u8AngleCorrection;
    UCHAR        u8CalVer[FACT_NEW_CAL_LEN];
    UCHAR        u8HardVer[FACT_NEW_HARD_LEN];
    UCHAR        u8ID[ID_LEN];
}tsSDKIDNew;

typedef struct tsSubsection
{
	float n1_org;              //原始n1值
	float n2_org;              //原始0n2值
	UINT8 ulower_0 = 0;         //分段标定第0段阈值
	UINT8 uupper_0 = 1;         //分段标定第0段阈值
	UINT8 ulower_1 = 1;         //分段标定第1段阈值
	UINT8 uupper_1 = 2;         //分段标定第1段阈值
	UINT8 ulower_2 = 2;         //分段标定第2段阈值
	UINT8 uupper_2 = 4;         //分段标定第2段阈值
	UINT8 ulower_3 = 4;         //分段标定第3段阈值
	UINT8 uupper_3 = 6;         //分段标定第3段阈值
	UINT8 ulower_4 = 6;         //分段标定第4段阈值
	UINT8 uupper_4 = 8;         //分段标定第4段阈值
	float n1_subsection_0;     //分段标定第0段n1值  
	float n2_subsection_0;     //分段标定第0段n2值
	float n1_subsection_1;     //分段标定第1段n1值  
	float n2_subsection_1;     //分段标定第1段n2值
	float n1_subsection_2;     //分段标定第2段n1值
	float n2_subsection_2;     //分段标定第2段n2值
	float n1_subsection_3;     //分段标定第3段n1值
	float n2_subsection_3;     //分段标定第3段n2值
	float n1_subsection_4;     //分段标定第4段n1值
	float n2_subsection_4;     //分段标定第4段n2值
	void reset()
	{
		n1_org = 0;
		n2_org = 0;
		ulower_0 = 0;
		uupper_0 = 1;
		ulower_1 = 1;
		uupper_1 = 2;
		ulower_2 = 2;
		uupper_2 = 4;
		ulower_3 = 4;
		uupper_3 = 6;
		ulower_4 = 6;
		uupper_4 = 8;
		n1_subsection_0 = 0;
		n2_subsection_0 = 0;
		n1_subsection_1 = 0;
		n2_subsection_1 = 0;
		n1_subsection_2 = 0;
		n2_subsection_2 = 0;
		n1_subsection_3 = 0;
		n2_subsection_3 = 0;
		n1_subsection_4 = 0;
		n2_subsection_4 = 0;
	}

}tsSubsection;



typedef struct tsSDKSN
{
	UINT16       u16Head;
	UINT16       u16Len;
	UINT16       u16Cal;
	UCHAR        u8Type;
	UCHAR        u8Ver;
	UCHAR        u8FacInfo[5];
	UCHAR        u8Reserve1[5];
	UCHAR        u8SpeedInfo[2];
	UCHAR        u8CalVer[3];
	UCHAR        u8Reserve2[6];
	UCHAR        u8HardVer[4];
	UINT16       u16Ang;
	UCHAR        u8Direction;
	UCHAR        u8AngleCorrection;
	UCHAR        u8Reserve3[4];
	UCHAR        u8SN[20];
}tsSDKSN;

typedef struct tsSDKIDD2M7
{
	tsSDKSN           sSN;
	tsSubsection      sSub;
}tsSDKIDD2M7;

typedef struct tsPackUID
{
	UINT16       u16Head;
	UINT16       u16Len;
	UINT16       u16Cal;
	UCHAR        u8Type;
	UCHAR        u8Ver;
	UCHAR        u8FacInfo[6];
	UINT16       u16A1;
	UINT16       u16B1;
	UINT16       u16Firmware;    
	UINT8        u8SoftVer[3];   
	UINT16       u16A2;
	UINT16       u16B2;
	UINT8        u8ConfigVer[2];  
	UINT8        u8HardwareVer[4]; 
	UINT16       u16AngleOffset;  //2 角度补偿（单位0.01度）
	UINT8        u8LDDirection;   
	UINT8        u8CorrectionEnble;  //
	UINT8        uReserved2[4];  //5 预留
	UINT8        uSN[20];      //
}tsPackUID;

typedef struct tsIDX2
{
    UINT16       u16Head;
    UCHAR        u8Ver[VER_LEN];
    UCHAR        u8ID[ID_LEN];
    UCHAR        u8Cal;
}tsIDX2;

typedef struct tsIDX1
{
    UINT16       u16Head;
    UCHAR        u8ID[ID_LEN];
    UINT16       u16Cal;
}tsIDX1;

typedef struct tsCmdInfo
{
    UINT16       u16Head;
    UINT16       u16Len;
    UINT16       u16Cal;
    UCHAR        u8Type;
    UCHAR        u8FacInfo[FAC_INFO_LEN];
    UINT16       u16Ang;
    UCHAR        u8Direction;
    UCHAR        u8AngleCorrection;
    UCHAR        u8ReserverInfo[RESERVER_LEN];
}tsCmdInfo;

typedef struct tsCmdStart
{
    UINT16       u16Head;
    UINT16       u16Len;
    UINT16       u16Cal;
    UCHAR        u8Type;
}tsCmdStart;

typedef struct tsPointCloudHead
{
    UINT16       u16Head;
    UCHAR        u8Info;
    UCHAR        u8Num;
    UINT16       u16Speed;
    UINT16       u16FirstAng;
}tsPointCloudHead;

typedef struct tsPointCloudHeadTof
{
	UINT16       u16Head;
	UCHAR        u8Info;
	UCHAR        u8Num;
	UINT16       u16Speed;
	UINT16       u16FirstAng;
	UINT16       u16LastAng;
	UINT16       u16Temperature;
}tsPointCloudHeadTof;

typedef struct tsBlockMessage
{
    UINT16       u16Head;
    UCHAR        u8Info;
    UCHAR        u8Len;
    UCHAR        u8MsgID;
    UCHAR        u8Code;
    UCHAR        u8Reserve;
    UCHAR        u8CheckSum;
}tsBlockMessage;


typedef struct tsPointCloudTail
{
    UINT16       u16LastAng;
    UINT16       u16CheckSum;
}tsPointCloudTail;

typedef struct tsPointCloudTailTof
{
	UINT16       u16CheckSum;
}tsPointCloudTailTof;

/*
typedef struct tsPointCloud
{
    UCHAR        u8Dist[DIST_BYTES];
    UINT16       u16Signal;
}tsPointCloud;
*/


#pragma pack(pop)


class HCHead
{
public:
    HCHead();

	static UINT64 getCurrentTimestampNs();
    static UINT64 getCurrentTimestampUs();
	static UINT64 getCurrentTimestampMs();

    static void eraseBuff(std::vector<UCHAR>& lstG,int iLen);
    static void eraseRangeData(LstPointCloud& lstG,int iLen);

	static double getAngleFromXY(const double x, const double y);
	static double getAngleFromAB(const double a, const double b);
	static double getDistFromAB(const double a, const double b);

	static float uint6_cov_float(UINT16 value);

	static UINT16 float_cov_uint16(float value);
};

bool nodeComparator(const tsNodeInfo& s1, const tsNodeInfo& s2);
bool newComparator(const tsPointCloud& s1, const tsPointCloud& s2);


static char*  print_curr_time()
{
	time_t now = time(nullptr);
	tm* curr_tm = localtime(&now);

	//tm curr_tm;
	//localtime_s(&curr_tm,&now);
	static char time[80] = { 0 };
	strftime(time, 80, "%Y-%m-%d %H:%M:%S    ", curr_tm);
	//printf(time);
	return time;
}


#ifdef __linux__
#define __FILENAME__ (strrchr(__FILE__, '/') ? (strrchr(__FILE__, '/') + 1):__FILE__)
#endif
#if defined (_WIN32) || defined( _WIN64)
#define __FILENAME__ (strrchr(__FILE__, '\\') ? (strrchr(__FILE__, '\\') + 1):__FILE__)
#endif

#define __MY_DATE__ (print_curr_time())




#define LOG_WARNING (printf("HCSDK W:%s %s:%u:\t", __MY_DATE__, __FILENAME__, __LINE__), printf) 
#define LOG_INFO    (printf("HCSDK I:%s %s:%u:\t", __MY_DATE__, __FILENAME__, __LINE__), printf) 
#define LOG_ERROR   (printf("HCSDK E:%s %s:%u:\t", __MY_DATE__, __FILENAME__, __LINE__), printf) 


#endif // HCHEAD_H
