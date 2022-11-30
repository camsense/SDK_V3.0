#ifndef _HCSDK_R_H_
#define _HCSDK_R_H_

#include "HcPointCloudData.h"

#ifdef __cplusplus
extern "C" {
#endif
    
	//Noise Tools interface

	/**************************************************************************************************************************/
    //�汾
	/**************************************************************************************************************************/
	char* GetVer();

	/**************************************************************************************************************************/
	//��ʼ�� 
	/**************************************************************************************************************************/
	bool hcSDKFltInitialize();	

	/**************************************************************************************************************************/
	//�ͷ�
	/**************************************************************************************************************************/
	bool hcSDKFltUnInit();	

	/**************************************************************************************************************************/
	// ����:  
	//      sLidarModel-eg.  "X2F" 
	//      bLowSpeed - true for Low Spin-Speed
	// ���:
	//      stFltLidarCfg - Lidar Paras 
	// Return:
	//      true: sLidarModel exisit in list,false: not in list
    /**************************************************************************************************************************/
	bool UpdateLidarPara(const char*	    sLidarModel, bool bLowSpeed, stFltLidarCfg_t &stFltLidarCfg);

	//**************************************************************************************************************************/
	//ͨ�ýӿں�������������,ͨ��eFltTypeѡ����������ͣ���ͨ��ǿ�⡢ͨ�ó�����
	//���� ��
	//		 lstPointCloud - 1 Ȧ��������
	//		 eFltType -  Filter ����ѡ��
	//		 stFltLidarCfg - Paras of �״�  
	//		 stFltGblSetting - Global ����
	//		 stFltLParas - Line filter ����
	//���:
	//		 lstPointCloud - �����ĵ�������
	//**************************************************************************************************************************/
	void hotPixelFilterGbl(std::vector<stPtCloud_t> &lstPointCloud,eFltType_t eFltType,const stFltLidarCfg_t &stFltLidarCfg,stFltGblSetting_t stFltGblSetting,stFltLParas_t stFltLParas);

	//**************************************************************************************************************************/
	//���泡���ӿں��������������˳� 
	//���룺
	//		 lstPointCloud - 1 circle point-clouds data
	//		 stFltLidarCfg - Paras of Lidar  
	//		 stFltGblSetting - Global Paras
	//		 stFltLParas - Line filter Paras
	//���:
	//		 lstPointCloud - the Modified point-clouds data
	//**************************************************************************************************************************/
	void hotPixelFilter(std::vector<stPtCloud_t> &lstPointCloud,const stFltLidarCfg_t &stFltLidarCfg,stFltGblSetting_t stFltGblSetting,stFltLParas_t stFltLParas);

	//**************************************************************************************************************************/
	//ǿ�ⳡ���ӿں��� 
	//���� ��
	//		 lstPointCloud - 1 Ȧ��������
	//		 stFltLidarCfg - Paras of �״�  
	//		 stFltGblSetting - Global ����
	//		 stFltLParas - Line filter ����
	//���:
	//		 lstPointCloud - �����ĵ�������
	//**************************************************************************************************************************/
	void strongLightFilter(std::vector<stPtCloud_t> &lstPointCloud,const stFltLidarCfg_t &stFltLidarCfg,stFltGblSetting_t stFltGblSetting,stFltLParas_t stFltLParas);

	/***********************************************************************************************************************************************************
	// ��vcCloudAIn �� vcCloudBIn �ǶȺ;��붼����ͬ�ĵ㣬��������vcCloudTsOut
	/************************************************************************************************************************************************************/
	void GetDifPtCloud(const std::vector <stFltCloudD_t> &vcCloudAIn,const std::vector <stFltCloudD_t> &vcCloudBIn,std::vector <stFltCloudD_t> &vcCloudTsOut);

	/***********************************************************************************************************************************************************
	// �� vcCloudTs ��ȥ�� vcToFltCloud����������vcCloudTs
	/************************************************************************************************************************************************************/
	void FltoutByPtCloud(std::vector <stPtCloud_t> &vcCloudTs,const std::vector <stFltCloudD_t> vcToFltCloud,bool bPtActiveSt);

	/***********************************************************************************************************************************************************
	// A + B -> C �����vcCloudAOut
	/************************************************************************************************************************************************************/
	void PlusPtCloud(const std::vector <stFltCloudD_t> &vcCloudAIn,const std::vector <stFltCloudD_t> &vcCloudBIn,std::vector <stFltCloudD_t> &vcCloudOut); 	            
	 											  
#ifdef __cplusplus
};
#endif

#endif#pragma once
