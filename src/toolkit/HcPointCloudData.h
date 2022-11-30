#pragma once
#include <vector>

using namespace std;
typedef unsigned short             UINT16;
typedef unsigned long long int     UINT64;

//////////////////////////////////////////////////////////////////////////////////
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
#define  X2BF                           "X2BF"
#define  X2C                            "X2C"
#define  X2D                            "X2D"
#define  X2E                            "X2E"
#define  X2F                            "X2F"
#define  X2M                            "X2M"
#define  X2MF                           "X2MF"
#define  X2N                            "X2N"
#define  X2YE                           "X2YE"
#define  X2YJ                           "X2YJ"
#define  X2YP                           "X2YP"
#define  D2A                            "D2A"
#define  D2B                            "D2B"

#define  T3A                            "T3A"
#define  T3B                            "T3B"
#define  T3C                            "T3C"
#define  D2A6                           "D2A6"
#define  D2A8                           "D2A8"
#define  D2M8                           "D2M8"
#define  D2M6                           "D2M6"
#define  D2B6                           "D2B6"
#define  D2B8                           "D2B8"
#define  D2S8                           "D2S8"
#define  D2S6                           "D2S6"
#define  D2P8                           "D2P8"
#define  D2P6                           "D2P6"


#define  BLND_DIST_X1                   100
#define  BLND_DIST_X2                   120
#define  BLND_DIST_D2                   140
#define  BLND_DIST_T3                   10


#define  FPS_1800_NOR                   1800
#define  FPS_2000_NOR                   2088 
#define  FPS_3000_NOR                   3048  
#define  FPS_TOF_NOR                    3300
#define  FPS_4000_NOR                   4000

#define  SPEED_180_NOR                  180
#define  SPEED_250_NOR                  250
#define  SPEED_300_NOR                  300
#define  SPEED_312_NOR                  312
#define  SPEED_315_NOR                  315
#define  SPEED_330_NOR                  330
#define  SPEED_360_NOR                  360
#define  SPEED_TOF_NOR                  360
#define  SPEED_420_NOR                  420


//////////////////////////////////////////////////////////////////////////////////
enum eFltType_t{FLT_NORMAL = 0,FLT_STRONGLIGHT_AND_OFF = 1,FLT_STRONGLIGHT_AND_NORMAL = 2,FLT_NOTHING = 3};

//用于定义 点云 类型，本toolkit 用于处理1圈点云数据
typedef struct _stPtCloud
{
	bool         bValid;                 // true:有效点, false:非有效点            
	double       dAngle;                 // 点云角度,degree     
	double       dAngleRaw;              // 初始点云角度,degree 
	double       dAngleDisp;             // 点云显示工具用变量，无用到    
	UINT16       u16Dist;                // 点云距离  ,mm  
	UINT16       u16DistRaw;             // 初始点云距离  ,mm
	UINT16       u16Speed;               // 雷达转速, RPM
	UINT16       u16Gray;                // 亮度
	bool         bGrayTwoByte;           // true：u16Gray存储2字节亮度信息,false u16Gray 存储1字节亮度信息 
	UINT64       u64TimeStampMs;         // 时间戳 ,ms  
	UINT64       u64TimeStampNs;    // timestamp ,ns  
	float        fTemperature;//温度
	bool         bOverRange;
	
	_stPtCloud() :
		bValid(true),
		dAngle(0.),
		dAngleRaw(0.),
		dAngleDisp(0.),
		u16Dist(0),
		u16DistRaw(0),
		u16Speed(0),
		u16Gray(0),
		bGrayTwoByte(false),
		u64TimeStampMs(0),
		u64TimeStampNs(0),
		fTemperature(0),
		bOverRange(false)
	{}
}stPtCloud_t;
typedef std::vector<stPtCloud_t> lstPtCloud_t;


typedef struct _FltCloudD                //Flt Cloud Data 
{
	double       x;                      //X 坐标 x = dist * cos(angle_rad_cur);
	double       y;                      //Y 坐标 y = dist * sin(angle_rad_cur);
  	UINT16       u16Dist;                //距离
  	double       dAngle;		         //原始角度 degree
	bool         bValidIs;	             //true:有效点 false:无效点
	int          nfound; 		         //程序动态标记

	_FltCloudD():
		x(65535),
		y(65535),
  		u16Dist(65535),          
  		dAngle(0),		    
		bValidIs(1),	    
		nfound(0)		
	{}	
}stFltCloudD_t; 


#define  DIST_INNER_CIRCLE      140      //mm Blind area size; eg. D2?:140, X2? 120, X1?100 
#define  LIDAR_FPS              3000     //帧率/second
#define  LIDAR_RPM              360      //转速/minute
#define  H_L_SPD_THRES          210      //Threshold of High/Low Spin-speed,design value;Eg. X2MF(180/360 L/H RMS) : 270,  X2F(360 RMS):270,D2S(300):210    

typedef struct _FltLidarCfg              //Lidar Paras 
{
  	unsigned int nBlindDist;             //盲区内被清除的距离 mm 	
	unsigned int nFPS;	                 //帧率/second  	
	unsigned int nSpinSpeed; 		     //转速/minute：设计指标，高低转速切换时，需更新此值
	int          nLHSpdThres;	         //Threshold of High/Low Spin-speed,design value;Eg. X2MF(180/360 L/H RMS) : 270,  X2F(360 RMS):270,D2S(300):210 
  	
  	_FltLidarCfg():
		nFPS(LIDAR_FPS),
		nSpinSpeed(LIDAR_RPM),	
		nBlindDist(DIST_INNER_CIRCLE),  		
		nLHSpdThres(H_L_SPD_THRES)
  	{}
}stFltLidarCfg_t; 


#define  ORG_REGION_L           65       //mm, Ray valid area,Better to opt.till now need not; 
#define  DIST_CLEAR_MAX_L       600      //mm, Line Clear Max,Better to opt       
#define  LINAR_OFST_MAX_L       3        //Max offset of Spot to the ray 
#define  LINAR_CNT_L            3        //Min Spots of Ray 

typedef struct _FltLParas                //L filter paras
{
	unsigned int nActDist;	             //mm,Ray-line 滤除有效范围，如600mm
	unsigned int nCirCenFld; 		     //mm,射线到原点距离阈值，如65mm
  	unsigned int nPtToLineMin;           //mm,同一直线点的距离阈值，如3mm
	unsigned int nPtOfLineMin;           //点->线最小点数阈值  	
  
	_FltLParas()
	{
		nActDist =   DIST_CLEAR_MAX_L;	   
		nCirCenFld = ORG_REGION_L; 		   
		nPtToLineMin = LINAR_OFST_MAX_L;   
		nPtOfLineMin = LINAR_CNT_L;          	          
	}	
}stFltLParas_t;


#define  SHARK_GRAY_THRES       50       //bGrayTwoByte true:400,false:50 
#define  HALOGEN_GRAY_THRES     7000     //Feature of Stronglight, need to adj., eg. bGrayTwoByte true:7000,false:255   
#define  ANG_COMP_OFST          50       //Angle offset value
#define  DIST_OUTER_CIRCLE      1000     //mm Range-limit,may Adjust as required
 
#define  FAC_HSPD_ADJ           3.0      //balance factor of high RPM, is to spot which is filtered out or left
#define  FAC_LSPD_ADJ           3.0      //balance factor of low RPM,is to spot which is filtered out or left

typedef struct _FltGblSetting
{
    unsigned int nActDist;	             //整体作用域 (单位: mm)
    unsigned int nHalogenGrayThres;      //7000+,Feature of Stronglight, need to adj., eg. bGrayTwoByte true:7000,false:255
	unsigned int nSharkGrayThres;        //bGrayTwoByte true:400,false:50 
	unsigned int nAngExtent;             //(+ Center -),Range degree,
	bool         bPtActiveSt;            //true or false, Valid Point Flag Setting
    float        fFacAdjHSpd;            //杂点滤除平衡系数-高转速
    float        fFacAdjLSpd;            //杂点滤除平衡系数-低转速
	bool         bFltRayLOn;             //true: RayLine Flt on, false:Off
	bool         bFlt1DotOn;             //true: 1 Dot Flt on, false:Off 
	bool         bFlt2DotOn;             //true: 1 Dot Flt on, false:Off, active in strong-light region
	int          nFltNDotOn;             //x>2: x Dots Flt on, 0: Off, active in strong-light region
	double       dActScopeSt;            //(0-360)Action is limited in the region of st-end,if dActScopeSt == dActScopeEnd mean the angle-limitation is invalid 
	double       dActScopeEnd;           //(0-360)Action is limited in the region of st-end,if dActScopeSt == dActScopeEnd mean the angle-limitation is invalid
	bool         bScopeSmallIs;          //true: acting in small region between dActScopeSt and dActScopeEnd, false: acting in big region between dActScopeSt and dActScopeEnd
	_FltGblSetting()
    {
		nActDist = DIST_OUTER_CIRCLE;	             
	   	nHalogenGrayThres = HALOGEN_GRAY_THRES;
	   	nSharkGrayThres = SHARK_GRAY_THRES;
	   	nAngExtent = ANG_COMP_OFST;
		bPtActiveSt = true;
		fFacAdjHSpd = FAC_HSPD_ADJ;
		fFacAdjLSpd = FAC_LSPD_ADJ;
        bFltRayLOn = true;    
		bFlt1DotOn = true;             
		bFlt2DotOn = true;            
		nFltNDotOn = 0;
	    dActScopeSt = 0.0;            
	    dActScopeEnd  = 0.0;          
	    bScopeSmallIs = true;           		
	}
}stFltGblSetting_t;


//////////////////////////////////////////////////////////////////////////////////	
enum ERR_CODE 
{
    LIDAR_SUC = 1,
    ERR_DATA_INPUT = 11,
    ERR_DATA_OUTPUT,
    ERR_ANGLE_PARA_NUM,
    ERR_DATA_REPAIR,

};
typedef struct _lidar_data_t 
{
    float     dAngle;
    UINT64    u64TimeStampMs;
    UINT16    u16Dist;
    UINT16    confidence;
} lidar_data_t;


