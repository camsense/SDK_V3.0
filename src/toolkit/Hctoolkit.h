#ifndef _HCSDK_R_H_
#define _HCSDK_R_H_

#include "HcPointCloudData.h"

#ifdef __cplusplus
extern "C" {
#endif
    
	//Noise Tools interface

	/**************************************************************************************************************************/
    //版本
	/**************************************************************************************************************************/
	char* GetVer();

	/**************************************************************************************************************************/
	//初始化 
	/**************************************************************************************************************************/
	bool hcSDKFltInitialize();	

	/**************************************************************************************************************************/
	//释放
	/**************************************************************************************************************************/
	bool hcSDKFltUnInit();	

	/**************************************************************************************************************************/
	// 输入:  
	//      sLidarModel-eg.  "X2F" 
	//      bLowSpeed - true for Low Spin-Speed
	// 输出:
	//      stFltLidarCfg - Lidar Paras 
	// Return:
	//      true: sLidarModel exisit in list,false: not in list
    /**************************************************************************************************************************/
	bool UpdateLidarPara(const char*	    sLidarModel, bool bLowSpeed, stFltLidarCfg_t &stFltLidarCfg);

	//**************************************************************************************************************************/
	//通用接口函数：作用域内,通过eFltType选择过滤器类型（普通、强光、通用场景）
	//输入 ：
	//		 lstPointCloud - 1 圈点云数据
	//		 eFltType -  Filter 类型选择
	//		 stFltLidarCfg - Paras of 雷达  
	//		 stFltGblSetting - Global 参数
	//		 stFltLParas - Line filter 参数
	//输出:
	//		 lstPointCloud - 处理后的点云数据
	//**************************************************************************************************************************/
	void hotPixelFilterGbl(std::vector<stPtCloud_t> &lstPointCloud,eFltType_t eFltType,const stFltLidarCfg_t &stFltLidarCfg,stFltGblSetting_t stFltGblSetting,stFltLParas_t stFltLParas);

	//**************************************************************************************************************************/
	//常规场景接口函数，作用域内滤除 
	//输入：
	//		 lstPointCloud - 1 circle point-clouds data
	//		 stFltLidarCfg - Paras of Lidar  
	//		 stFltGblSetting - Global Paras
	//		 stFltLParas - Line filter Paras
	//输出:
	//		 lstPointCloud - the Modified point-clouds data
	//**************************************************************************************************************************/
	void hotPixelFilter(std::vector<stPtCloud_t> &lstPointCloud,const stFltLidarCfg_t &stFltLidarCfg,stFltGblSetting_t stFltGblSetting,stFltLParas_t stFltLParas);

	//**************************************************************************************************************************/
	//强光场景接口函数 
	//输入 ：
	//		 lstPointCloud - 1 圈点云数据
	//		 stFltLidarCfg - Paras of 雷达  
	//		 stFltGblSetting - Global 参数
	//		 stFltLParas - Line filter 参数
	//输出:
	//		 lstPointCloud - 处理后的点云数据
	//**************************************************************************************************************************/
	void strongLightFilter(std::vector<stPtCloud_t> &lstPointCloud,const stFltLidarCfg_t &stFltLidarCfg,stFltGblSetting_t stFltGblSetting,stFltLParas_t stFltLParas);

	/***********************************************************************************************************************************************************
	// 将vcCloudAIn 和 vcCloudBIn 角度和距离都不相同的点，结果输出到vcCloudTsOut
	/************************************************************************************************************************************************************/
	void GetDifPtCloud(const std::vector <stFltCloudD_t> &vcCloudAIn,const std::vector <stFltCloudD_t> &vcCloudBIn,std::vector <stFltCloudD_t> &vcCloudTsOut);

	/***********************************************************************************************************************************************************
	// 从 vcCloudTs 中去除 vcToFltCloud，结果输出到vcCloudTs
	/************************************************************************************************************************************************************/
	void FltoutByPtCloud(std::vector <stPtCloud_t> &vcCloudTs,const std::vector <stFltCloudD_t> vcToFltCloud,bool bPtActiveSt);

	/***********************************************************************************************************************************************************
	// A + B -> C 结果：vcCloudAOut
	/************************************************************************************************************************************************************/
	void PlusPtCloud(const std::vector <stFltCloudD_t> &vcCloudAIn,const std::vector <stFltCloudD_t> &vcCloudBIn,std::vector <stFltCloudD_t> &vcCloudOut); 	            
	 											  
#ifdef __cplusplus
};
#endif

#endif#pragma once
