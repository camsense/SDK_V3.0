#pragma once
#ifndef HCLIDAR_FIL_H
#define HCLIDAR_FIL_H

//#include "extern.h"

#include <string>
#include <sstream>
#include <iostream>
#include <fstream>

#include <thread>
#include <mutex>
#include <cmath>
#include <list>
#include <vector>
#include <chrono>
#include <condition_variable>
#include <atomic>
#include <functional>
#include <map>
#include "HcPointCloudData.h"

#define TOOLKIT_VER  "1.0.0"

#define CV_PI   3.1415926535897932384626433832795
class HCLidarFilter
{
public:
    ~HCLidarFilter() {}
	HCLidarFilter()
	{	
		nLastSpd = 0;
		nFltRobustChk = 0;
		dAngStLast=0x1e1e;	
		dAngEndLast=0x1e1e; 
	}
    static HCLidarFilter& getInstance()
    {
        static HCLidarFilter instance;  
        return instance;
    }

    bool initialize()
	{
		return true;
	}
    bool unInit()
	{
		return true;
	}

private:
	std::mutex			m_mutex; 
	lstPtCloud_t		m_pointCloud;
	int 				nLastSpd;
	int 				nFltRobustChk;
	double				dAngStLast; 
	double				dAngEndLast; 

public:
	char* GetVer() { return TOOLKIT_VER; }
	void GetDifPtCloud(const std::vector <stFltCloudD_t> &vcCloudAIn,const std::vector <stFltCloudD_t> &vcCloudBIn,std::vector <stFltCloudD_t> &vcCloudTsOut);									   
	void FltoutByPtCloud(std::vector <stPtCloud_t> &vcCloudTs,const std::vector <stFltCloudD_t> &vcToFltCloud,bool bPtActiveSt);	
	void PlusPtCloud(const std::vector <stFltCloudD_t> &vcCloudAIn,const std::vector <stFltCloudD_t> &vcCloudBIn,std::vector <stFltCloudD_t> &vcCloudOut);  											 
	void hotPixelFilterGbl(lstPtCloud_t& lstPointCloud,eFltType_t eFltType,const stFltLidarCfg_t &stFltLidarCfg,stFltGblSetting_t stFltGblSetting,stFltLParas_t stFltLParas);
	void hotPixelFilter(lstPtCloud_t& lstPointCloud,const stFltLidarCfg_t &stFltLidarCfg,stFltGblSetting_t stFltGblSetting,stFltLParas_t stFltLParas);	
	void strongLightFilter(lstPtCloud_t& lstPointCloud,const stFltLidarCfg_t &stFltLidarCfg,stFltGblSetting_t stFltGblSetting,stFltLParas_t stFltLParas); 
	bool UpdateLidarPara(const std::string		  sLidarModel, bool bLowSpeed, stFltLidarCfg_t &stFltLidarCfg);

private:
	void strongLightFilterPre(lstPtCloud_t &lstPointCloud,double  &dAngleBegin,double &          dAngleEnd,bool &bEnAngLmt,const stFltLidarCfg_t &stFltLidarCfg,stFltGblSetting_t &stFltGblSetting,stFltLParas_t stFltLParas);
	bool angleInRange(const double dAngleIn, const double &dAngSt , const double &dAngEnd );
	void GetDistDotThres(unsigned int nDist,double &dDistOfPtFac, double &dCaliFac, double &dThresSet);
	void ScopeSeek(lstPtCloud_t &vcPoint,double &dAngSt, double &dAngEnd,bool bEnClrAng);	
	void checkPoint(lstPtCloud_t &lstPointCloud, eFltType_t eFltType, double dAngSt, double dAngEnd, bool bEnAngLmt,const stFltLidarCfg_t &stFltLidarCfg,stFltGblSetting_t stFltGblSetting,stFltLParas_t stFltLParas);		
	void hotPixelCheck(lstPtCloud_t &lstPointCloud,const stFltLidarCfg_t &stFltLidarCfg,stFltGblSetting_t stFltGblSetting,stFltLParas_t stFltLParas,eFltType_t eFltType= FLT_NORMAL,double dAngSt=0, double dAngEnd=0);
	void FltL(lstPtCloud_t &vcPoint,vector<stFltCloudD_t> &vcPointData,const double &dAngSt,const double &dAngEnd,stFltLParas_t stFltLParas);
	void Flt1Pt(lstPtCloud_t &vcPoint,vector<stFltCloudD_t> &vcPointData,const double &dAngSt,const double &dAngEnd, int giFilterSpeed,int giFilterFPS,float fFacAdj);
	void Flt2Pt(lstPtCloud_t &vcPoint,vector<stFltCloudD_t> &vcPointData,const double &dAngSt,const double &dAngEnd, int giFilterSpeed,int giFilterFPS,float fFacAdj,bool bEnAngLmt);			
	void FltNPt(lstPtCloud_t &vcPoint,vector<stFltCloudD_t> &vcPointData,const double &dAngSt,const double &dAngEnd, int giFilterSpeed,int giFilterFPS,float fFacAdj,int nTrace,bool bEnAngLmt);
	
protected: //非正式的预留代码
	void FltN(std::vector<stFltCloudD_t> &data_in,std::vector<stFltCloudD_t> &data_out,std::vector<stFltCloudD_t> &flted_out,const stFltLidarCfg_t &stLidarPara,
		             stFltGblSetting_t stFltGblSetting, bool bFlt1,float fDotFactor,bool bEmptyBlindA,bool bPreAct);
    void FltRawN(const std::vector<stPtCloud_t> data_in,std::vector<stPtCloud_t> &data_out,const stFltLidarCfg_t &stLidarPara,
                         stFltGblSetting_t stFltGblSetting,float fSeekAngSt,float fSeekAngEnd,bool bFlt1,float fDotFactor,bool bEmptyBlindA,bool bPreAct);		
	void FltRawN(const std::vector<stPtCloud_t> &data_in,std::vector<stFltCloudD_t> &act_out,std::vector<stFltCloudD_t> &flted_out,const stFltLidarCfg_t &stLidarPara,
                         stFltGblSetting_t stFltGblSetting,float fSeekAngSt,float fSeekAngEnd,bool bFlt1,float fDotFactor,bool bEmptyBlindA,bool bPreAct);
	void FltLn(std::vector<stFltCloudD_t> &act_in,std::vector<stFltCloudD_t> &act_out,std::vector<stFltCloudD_t> &flted_out,const stFltLidarCfg_t &stLidarPara,
                         stFltGblSetting_t stFltGblSetting,stFltLParas_t stFltLParas,bool bEmptyBlindA,bool bPreAct);	
	void FltRawL(const std::vector<stPtCloud_t> data_in,std::vector<stPtCloud_t> &data_out,const stFltLidarCfg_t &stLidarPara,
                         stFltGblSetting_t stFltGblSetting,stFltLParas_t stFltLParas,float fSeekAngSt,float fSeekAngEnd,bool bEmptyBlindA,bool bPreAct);		
	void FltRawL(const std::vector<stPtCloud_t> data_in,std::vector<stFltCloudD_t> &act_out,std::vector<stFltCloudD_t> &flted_out,const stFltLidarCfg_t &stLidarPara,
                         stFltGblSetting_t stFltGblSetting,stFltLParas_t stFltLParas,float fSeekAngSt,float fSeekAngEnd,bool bEmptyBlindA,bool bPreAct);	
	void FltHalogen(const std::vector<stPtCloud_t> data_in,std::vector<stPtCloud_t> &data_out,const stFltLidarCfg_t &stLidarPara,
                         const stFltGblSetting_t &stFltGblSetting,stFltLParas_t &stFltLParas,float fDotFactor,bool bFlt1,bool bEmptyBlindA,bool bPreAct);

};

#endif 
