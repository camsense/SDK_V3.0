//#include "pch.h"
#include "Hctoolkit.h"
#include <string>
#include <iostream>
#include "HcPointCloudData.h"
#include "lidar_repair.h"
#include "lidar_filter.h"


#ifdef __cplusplus
extern "C" {
#endif

    //Ray,Noise,stronglight 
  
	HCLidarFilter& g_LidarFlt = HCLidarFilter::getInstance();
    //版本信息
	char* GetVer()
    {
        return g_LidarFlt.GetVer();
    }
    //初始化
	bool hcSDKFltInitialize()
	{
		 return g_LidarFlt.initialize();
	}
	//释放
	bool hcSDKFltUnInit()
	{
		 return g_LidarFlt.unInit();
	}
	
	void hotPixelFilterGbl(std::vector<stPtCloud_t> &lstPointCloud,eFltType_t eFltType,const stFltLidarCfg_t &stFltLidarCfg,stFltGblSetting_t stFltGblSetting,stFltLParas_t stFltLParas)
	{
		g_LidarFlt.hotPixelFilterGbl(lstPointCloud, eFltType,stFltLidarCfg,stFltGblSetting,stFltLParas);
	}										 
	void hotPixelFilter(std::vector<stPtCloud_t> &lstPointCloud,const stFltLidarCfg_t &stFltLidarCfg,stFltGblSetting_t stFltGblSetting,stFltLParas_t stFltLParas)
	{
		g_LidarFlt.hotPixelFilter(lstPointCloud,stFltLidarCfg,stFltGblSetting,stFltLParas);
	}		
	void strongLightFilter(std::vector<stPtCloud_t> &lstPointCloud,const stFltLidarCfg_t &stFltLidarCfg,stFltGblSetting_t stFltGblSetting,stFltLParas_t stFltLParas)
	{
		g_LidarFlt.strongLightFilter(lstPointCloud,stFltLidarCfg,stFltGblSetting,stFltLParas);
	}

	bool UpdateLidarPara(const char*        sLidarModel, bool bLowSpeed, stFltLidarCfg_t &stFltLidarCfg)
	{
		return g_LidarFlt.UpdateLidarPara(sLidarModel, bLowSpeed, stFltLidarCfg);
	}
	void GetDifPtCloud(const std::vector <stFltCloudD_t> &vcCloudAIn,const std::vector <stFltCloudD_t> &vcCloudBIn,std::vector <stFltCloudD_t> &vcCloudTsOut)
	{
		g_LidarFlt.GetDifPtCloud(vcCloudAIn,vcCloudBIn,vcCloudTsOut);
	}
	void FltoutByPtCloud(std::vector <stPtCloud_t> &vcCloudTs,const std::vector <stFltCloudD_t> vcToFltCloud,bool bPtActiveSt)
	{
		g_LidarFlt.FltoutByPtCloud(vcCloudTs,vcToFltCloud,bPtActiveSt);
	}
	void PlusPtCloud(const std::vector <stFltCloudD_t> &vcCloudAIn,const std::vector <stFltCloudD_t> &vcCloudBIn,std::vector <stFltCloudD_t> &vcCloudOut) 
	{
		g_LidarFlt.PlusPtCloud(vcCloudAIn,vcCloudBIn,vcCloudOut);
	}

    // Pillar Gap 
    HCLidarRepair& g_LidarGap = HCLidarRepair::getInstance();

    bool hcSDKRepairInitialize()
	{
		return g_LidarGap.initialize();
	}
	bool hcSDKRepairUnInit()
	{
		return g_LidarGap.unInit();
	}
	UINT16 RepairPointCloud(const std::vector<_lidar_data_t> data_in, std::vector<_lidar_data_t>& data_out, int dAngle[][2], int nParaNum)
	{
		return g_LidarGap.RepairPointCloud(data_in, data_out, dAngle, nParaNum);
	}	
	
#ifdef __cplusplus
};
#endif
