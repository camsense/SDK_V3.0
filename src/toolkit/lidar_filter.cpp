#include "lidar_filter.h"
#include <math.h>
#include <iostream>
#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>


#define  EN_THRES_CALIBRTE    1       //
#define  EN_INHERIT           1       //function option
#define  EN_ANG_LIM           1       //function with angle limit


/**************************************************************************************************************************/
//距离quick sort 
/**************************************************************************************************************************/
bool comP2DDist(stFltCloudD_t&a, stFltCloudD_t&b)
{
	if (a.u16Dist > b.u16Dist)
	{
		return true;
	}
	else
	{
		return false;
	}
}
/**************************************************************************************************************************/
//角度quick sort,H->L 
/**************************************************************************************************************************/
bool comP2DAng(stFltCloudD_t& s1, stFltCloudD_t& s2)
{
	return s1.dAngle > s2.dAngle;
}


/**************************************************************************************************************************/
//光强quick sort,H->L 
/**************************************************************************************************************************/
bool comTSGray(stPtCloud_t&a, stPtCloud_t&b)
{
	if (a.u16Gray > b.u16Gray)
	{
		return true;
	}
	else
	{
		return false;
	}
}
/**************************************************************************************************************************/
//角度quick sort H->L 
/**************************************************************************************************************************/
bool comTSAng(const stPtCloud_t& s1, const stPtCloud_t& s2)

{
	return s1.dAngle > s2.dAngle;
}
//**************************************************************************************************************************/
//常规场景接口函数，作用域内滤除 
//输入：
//       lstPointCloud - 1 circle point-clouds data
//       stFltLidarCfg - Paras of Lidar  
//       stFltGblSetting - Global Paras
//       stFltLParas - Line filter Paras
//输出:
//       lstPointCloud - the Modified point-clouds data
//**************************************************************************************************************************/
void HCLidarFilter::hotPixelFilter(lstPtCloud_t &lstPointCloud,const stFltLidarCfg_t &stFltLidarCfg,stFltGblSetting_t stFltGblSetting,stFltLParas_t stFltLParas) 
{	
	m_mutex.lock();	

	if(stFltGblSetting.dActScopeEnd >= 360)
	{
		stFltGblSetting.dActScopeEnd = stFltGblSetting.dActScopeEnd - 360;
	}
	if(stFltGblSetting.dActScopeSt < 0)
	{
		stFltGblSetting.dActScopeSt = 360 + stFltGblSetting.dActScopeSt;
	}

    if(stFltGblSetting.dActScopeSt > stFltGblSetting.dActScopeEnd)
    {         
		double dtemp = stFltGblSetting.dActScopeSt;
		stFltGblSetting.dActScopeSt = stFltGblSetting.dActScopeEnd;
		stFltGblSetting.dActScopeEnd = dtemp;    
	}	
	if(stFltLidarCfg.nBlindDist)
	{
		lstPtCloud_t::iterator it;
		for (it = lstPointCloud.begin(); it != lstPointCloud.end();)
		{  
			if (it->u16Dist && it->u16Dist <= stFltLidarCfg.nBlindDist)
			{
				it = lstPointCloud.erase(it);
			}
			else
				it++;
		} 
    }
	hotPixelCheck(lstPointCloud,stFltLidarCfg,stFltGblSetting,stFltLParas);    
	m_mutex.unlock();		
}

//**************************************************************************************************************************/
//通用接口函数：作用域内,通过eFltType选择过滤器类型（普通、强光、通用场景）
//输入 ：
//       lstPointCloud - 1 圈点云数据
//       eFltType -  Filter 类型选择
//       stFltLidarCfg - Paras of 雷达  
//       stFltGblSetting - Global 参数
//       stFltLParas - Line filter 参数
//输出:
//       lstPointCloud - 处理后的点云数据
//**************************************************************************************************************************/
void HCLidarFilter::hotPixelFilterGbl(lstPtCloud_t &lstPointCloud,eFltType_t eFltType,const stFltLidarCfg_t &stFltLidarCfg,stFltGblSetting_t stFltGblSetting,stFltLParas_t stFltLParas) 
{	
    if(eFltType == FLT_NORMAL)
    {
		hotPixelFilter(lstPointCloud,stFltLidarCfg,stFltGblSetting,stFltLParas);
    }
	else
	{
		m_mutex.lock(); 	
		double dAngleBegin,dAngleEnd;
		bool bEnAngLmt;
		strongLightFilterPre(lstPointCloud, dAngleBegin, dAngleEnd,bEnAngLmt, stFltLidarCfg, stFltGblSetting,stFltLParas);
		checkPoint(lstPointCloud, eFltType, dAngleBegin, dAngleEnd,bEnAngLmt,stFltLidarCfg,stFltGblSetting,stFltLParas);		
		m_mutex.unlock();	
	}
}

//**************************************************************************************************************************/
//强光场景接口函数 
//输入 ：
//       lstPointCloud - 1 圈点云数据
//       stFltLidarCfg - Paras of 雷达  
//       stFltGblSetting - Global 参数
//       stFltLParas - Line filter 参数
//输出:
//       lstPointCloud - 处理后的点云数据
//**************************************************************************************************************************/
void HCLidarFilter::strongLightFilter(lstPtCloud_t& lstPointCloud,const stFltLidarCfg_t &stFltLidarCfg,stFltGblSetting_t stFltGblSetting,stFltLParas_t stFltLParas) 
{	
	m_mutex.lock();	
	double dAngleBegin,dAngleEnd;
	bool bEnAngLmt;
	strongLightFilterPre(lstPointCloud, dAngleBegin, dAngleEnd,bEnAngLmt, stFltLidarCfg, stFltGblSetting,stFltLParas);
	checkPoint(lstPointCloud, FLT_STRONGLIGHT_AND_OFF, dAngleBegin, dAngleEnd,bEnAngLmt,stFltLidarCfg,stFltGblSetting,stFltLParas);		
	m_mutex.unlock();	
}

/**************************************************************************************************************************/
// Stronglight预处理函数
// 输入: 
//       lstPointCloud - 1 圈点云数据
// 输出:
//   lstPointCloud - 处理后的点云数据
//   dAngleBegin - Start Angle of Strong-light
//   dAngleEnd - End Angle of Strong-light
//   bEnAngLmt - Angle limitation flag 
/**************************************************************************************************************************/
#define  GRAY_DIST_LIMIT      800      //Distance to get Halogen light, if need to adj.;
#define  HALOGEN_DETECT_ADJ   0        //mm dist ext. to find halogen feature,if with Blind-Area type. if need, to adj.

void HCLidarFilter::strongLightFilterPre(lstPtCloud_t &lstPointCloud,double  &dAngleBegin,double &          dAngleEnd,bool &bEnAngLmt,const stFltLidarCfg_t &stFltLidarCfg,stFltGblSetting_t &stFltGblSetting,stFltLParas_t stFltLParas)
{	
	if(stFltGblSetting.dActScopeEnd >= 360)
	{
		stFltGblSetting.dActScopeEnd = stFltGblSetting.dActScopeEnd - 360;
	}
	if(stFltGblSetting.dActScopeSt < 0)
	{
		stFltGblSetting.dActScopeSt = 360 + stFltGblSetting.dActScopeSt;
	}

    if(stFltGblSetting.dActScopeSt > stFltGblSetting.dActScopeEnd)
    {         
		double dtemp = stFltGblSetting.dActScopeSt;
		stFltGblSetting.dActScopeSt = stFltGblSetting.dActScopeEnd;
		stFltGblSetting.dActScopeEnd = dtemp;    
	}	
   	
	bool    bStrongIs = false;
	bool    bStrongCurIs = false;
	int     nStrongCnt = 0;
	int     nLmax = 0, nLMaxInner = 0;
	int     nBegin = 0, nEnd = 0;
	bool    bFirst = false;

	double  dInnerAngSt = 0.0;
	double  dInnerAngEnd = 0.0;
	bool    bByHighLight = false;

	dAngleBegin = 0;
	dAngleEnd = 0;
	stPtCloud_t sInfo;
	
    bEnAngLmt = true;
	int biS[4]={0,0,0,0};
	for (int i = 0; i < lstPointCloud.size(); i++)
	{
		sInfo = lstPointCloud[i];
		if (nLmax < sInfo.u16Gray)
			nLmax = sInfo.u16Gray;
		
		if(sInfo.u16Dist && sInfo.u16Dist < GRAY_DIST_LIMIT)
		{            
		   if(stFltGblSetting.dActScopeSt == stFltGblSetting.dActScopeEnd
			   ||((stFltGblSetting.dActScopeSt != stFltGblSetting.dActScopeEnd)
			   &&(stFltGblSetting.bScopeSmallIs && angleInRange(lstPointCloud[i].dAngle, stFltGblSetting.dActScopeSt, stFltGblSetting.dActScopeEnd) 
			      || stFltGblSetting.bScopeSmallIs==false && angleInRange(lstPointCloud[i].dAngle, stFltGblSetting.dActScopeSt, stFltGblSetting.dActScopeEnd)==false)))
		   	{
				if (sInfo.u16Gray >= stFltGblSetting.nHalogenGrayThres)
				{
					nStrongCnt++;
					bByHighLight = true;
				}
				else  if (sInfo.u16Dist && sInfo.u16Dist <= (stFltLidarCfg.nBlindDist + HALOGEN_DETECT_ADJ)  && sInfo.u16Gray >= stFltGblSetting.nSharkGrayThres)
				{
					if (dInnerAngSt == 0)
						dInnerAngSt = sInfo.dAngle;
					
				    if (dInnerAngEnd == 0)
				    	dInnerAngEnd = sInfo.dAngle;
					else
					{
				    	if(dInnerAngEnd > sInfo.dAngle) 
					    	dInnerAngEnd = sInfo.dAngle;
					}				
					nStrongCnt++;

					if (nLMaxInner < sInfo.u16Gray)
						nLMaxInner = sInfo.u16Gray;

					if (sInfo.dAngle>=0 &&  sInfo.dAngle <= 90)
						biS[0]++;
					else if (sInfo.dAngle>90 &&	sInfo.dAngle <= 180)
						biS[1]++;
					else if (sInfo.dAngle>180 &&  sInfo.dAngle <= 270)
						biS[2]++;
					else if (sInfo.dAngle>270 &&  sInfo.dAngle <= 360)
						biS[3]++;							
				}
			}
		}
	}

	if(nStrongCnt >= 1)
	{
		bStrongIs = true;
	}
    	
	double dCenter = 0.0;		
	if (bStrongIs && bByHighLight == false)  
	{            
		if(dInnerAngEnd > dInnerAngSt)
		{
			dAngleBegin = dInnerAngSt;
			dAngleEnd = dInnerAngEnd;
		}
		else
		{
			dAngleBegin = dInnerAngEnd;
			dAngleEnd = dInnerAngSt;
		}

    	int iQuadrant =0;
	   	if(biS[0])
        	iQuadrant++;
	   	if(biS[1])
          	iQuadrant++;
	   	if(biS[2])
          	iQuadrant++;
	   	if(biS[3])
          	iQuadrant++;

	   	if(iQuadrant == 3)
	   	{
			if(biS[0]==0)
			{
				if(biS[1]>biS[3])
				{
					//180
					 
				}					
				else
				{
                   //270
				}
			}
			else if(biS[1]==0)
			{
				if(biS[0]>biS[2])
				{
					//0
				}					
				else
				{
                   //270
				}			
			}
			else if(biS[2]==0)
			{
				if(biS[1]>biS[3])
				{
					//90
				}					
				else
				{
                   //0
				}			
			}
			else  //if(biS[3]==0)
			{
				if(biS[0]>biS[2])
				{
					//90
				}					
				else
				{
                   //180
				}			
			}
	   	}
         
		if(iQuadrant >= 3)
		{
			bEnAngLmt = false;
		}
					
	}
	else 
	{
		lstPtCloud_t tmp0(lstPointCloud);
        sort(tmp0.begin(), tmp0.end(), comTSGray);   
		dCenter = fmod(tmp0[0].dAngle, 360.0);		
		dAngleBegin = dCenter;
		dAngleEnd = dCenter;

		if (dAngleEnd > 360)
		{
			dAngleEnd = dAngleEnd - 360;
		}
		if (dAngleBegin < 0)
		{
			dAngleBegin = 360 + dAngleBegin;
		}
		
		if(dAngleBegin>dAngleEnd)
		{
			double dtemp = dAngleBegin;
			dAngleBegin = dAngleEnd;
			dAngleEnd = dtemp;				
		}							
	}
	
	
	if (bStrongIs == false) 
	{
		dAngleBegin = 0;
		dAngleEnd = 0;
	}

	if(stFltLidarCfg.nBlindDist)
	{
		lstPtCloud_t::iterator it;
		for (it = lstPointCloud.begin(); it != lstPointCloud.end();)
		{  

			if (it->u16Dist && it->u16Dist <= stFltLidarCfg.nBlindDist)
			{
				it = lstPointCloud.erase(it);
			}
			else
				it++;
		} 
    }	
}


/**************************************************************************************************************************/
// 滤除检测函数 
// 输入: 
//       lstPointCloud - 1 圈点云数据
//       eFltType -  Filter 类型选择
//       dAngleBegin - Start Angle of Strong-light
//       dAngleEnd - End Angle of Strong-light
//       bEnAngLmt - Angle limitation flag for stronglight         
//       stFltLidarCfg - Paras of 雷达  
//       stFltGblSetting - Global 参数
//       stFltLParas - Line filter 参数
//输出:
//       lstPointCloud - 处理后的点云数据
//**************************************************************************************************************************/
#define  CONFIDENCE_THR      7        //confidence level
void HCLidarFilter::checkPoint(lstPtCloud_t &lstPointCloud, eFltType_t eFltType, double dAngSt, double dAngEnd, bool bEnAngLmt,const stFltLidarCfg_t &stFltLidarCfg,stFltGblSetting_t stFltGblSetting,stFltLParas_t stFltLParas)
{
	if(eFltType == FLT_NOTHING)  //Do nothing
	{
		return;
	}

    bool bHSpd = false;
	float fFacAdj = stFltGblSetting.fFacAdjHSpd;	
    if(nLastSpd==0)   
    	nLastSpd = stFltLidarCfg.nSpinSpeed;
    else
    {      
		if(nLastSpd > stFltLidarCfg.nLHSpdThres)  
		{
			bHSpd = true;
			fFacAdj = stFltGblSetting.fFacAdjHSpd;       
		}
		else 
		{
			bHSpd = false;
			fFacAdj = stFltGblSetting.fFacAdjLSpd;
		}
		
		if(nLastSpd != stFltLidarCfg.nSpinSpeed)  
		{
			dAngStLast = 0x1e1e;
			dAngEndLast = 0x1e1e;
		}			 
		nLastSpd = stFltLidarCfg.nSpinSpeed;
	}
 
	if((dAngSt||dAngEnd)) 
	{
	    double dAngStEst = dAngSt;
		double dAngEndEst = dAngEnd;
        if(dAngStLast == 0x1e1e)
			dAngStLast = dAngSt;
        if(dAngEndLast == 0x1e1e)
			dAngEndLast = dAngEnd;
 		nFltRobustChk = CONFIDENCE_THR;  //degree confidence		
           
        if(dAngStEst == dAngEndEst)  // ==
        {
			if( (dAngStEst-1) >= 0 )
				dAngStEst -= 1;			
			else if( (dAngStEst+1) < 360 )
			{
				dAngEndEst += 1;
			}
		}
		   		
		if(dAngStEst <= 180 && fabs(360-dAngEndEst + dAngStEst)<181)			
        {						
			dAngStEst = dAngStEst+stFltGblSetting.nAngExtent;
			dAngEndEst = dAngEndEst-stFltGblSetting.nAngExtent;
									
			if (dAngStEst > 360) 
			{
				dAngStEst = dAngStEst - 360;
			}
			if (dAngEndEst < 0)
			{
				dAngEndEst = 360 + dAngEndEst;
			}
			  
			dAngStLast = dAngStEst; 	
			dAngEndLast = dAngEndEst;
			
		}
		else //continue
		{								
			dAngStEst = dAngStEst-stFltGblSetting.nAngExtent;
			dAngEndEst = dAngEndEst+stFltGblSetting.nAngExtent;
			
			if (dAngStEst > 360)
			{
				dAngStEst = dAngStEst - 360;
			}
			if (dAngStEst < 0)
			{
				dAngStEst = 360 + dAngStEst;
			}	

			dAngStLast = dAngStEst; 				
			dAngEndLast = dAngEndEst;			
		}	
		
        if(dAngStLast > dAngEndLast)  //保证 Start < End 
        {
			double dtmp = dAngStLast;
			dAngStLast = dAngEndLast;
			dAngEndLast = dtmp;
        }
 	}
	else  
	{
		if(nFltRobustChk)	  
			nFltRobustChk--;
        
		if(nFltRobustChk==0)
		{
			dAngStLast = 0x1e1e;
			dAngEndLast = 0x1e1e;
		}			
	}
	
	vector<stFltCloudD_t> vcPointData;
	vcPointData.clear();
	for (int i = 0; i < lstPointCloud.size(); i++)
	{
		if (lstPointCloud[i].u16Dist && lstPointCloud[i].u16Dist <= stFltGblSetting.nActDist)
		{   		
		    if(stFltGblSetting.dActScopeSt == stFltGblSetting.dActScopeEnd
               ||((stFltGblSetting.dActScopeSt != stFltGblSetting.dActScopeEnd)
				&& (stFltGblSetting.bScopeSmallIs && angleInRange(lstPointCloud[i].dAngle, stFltGblSetting.dActScopeSt, stFltGblSetting.dActScopeEnd) 
				|| stFltGblSetting.bScopeSmallIs==false && angleInRange(lstPointCloud[i].dAngle, stFltGblSetting.dActScopeSt, stFltGblSetting.dActScopeEnd)==false)))
			{
				stFltCloudD_t pTemp;
				double angle_rad_cur = lstPointCloud[i].dAngle * CV_PI / 180;
				unsigned short u16Dist = lstPointCloud[i].u16Dist;
				pTemp.x = u16Dist * cos(angle_rad_cur);
				pTemp.y = u16Dist * sin(angle_rad_cur);
				pTemp.u16Dist = u16Dist;
				pTemp.dAngle = lstPointCloud[i].dAngle;
				pTemp.nfound = 0;
				vcPointData.push_back(pTemp);
			}
		}					
	}    		  
	sort(vcPointData.begin(), vcPointData.end(), comP2DAng); 

    lstPtCloud_t &vcPoint = lstPointCloud;  //lstPtCloud_t vcPoint(lstPointCloud);
	
	if(eFltType == FLT_STRONGLIGHT_AND_OFF) //Strong light + Off-FLT
	{
		if(nFltRobustChk && stFltGblSetting.bFltRayLOn)
			FltL(vcPoint,vcPointData,dAngStLast, dAngEndLast, stFltLParas);
		
	    if(stFltGblSetting.bFlt1DotOn)
	    	Flt1Pt(vcPoint,vcPointData,dAngStLast, dAngEndLast, stFltLidarCfg.nSpinSpeed,stFltLidarCfg.nFPS,fFacAdj); 		

		if(nFltRobustChk)
		{	
			if(stFltGblSetting.bFlt2DotOn)			
				Flt2Pt(vcPoint,vcPointData,dAngStLast, dAngEndLast, stFltLidarCfg.nSpinSpeed,stFltLidarCfg.nFPS,fFacAdj,bEnAngLmt); 	 
			
			if(stFltGblSetting.nFltNDotOn > 2)
				FltNPt(vcPoint,vcPointData,dAngStLast, dAngEndLast, stFltLidarCfg.nSpinSpeed,stFltLidarCfg.nFPS,fFacAdj,stFltGblSetting.nFltNDotOn,bEnAngLmt);
		}	
	}
	else if(eFltType == FLT_STRONGLIGHT_AND_NORMAL) //Strong light + Normal-Flt 
	{   	    
		if(stFltGblSetting.bFltRayLOn || nFltRobustChk)  
			FltL(vcPoint,vcPointData,dAngStLast, dAngEndLast, stFltLParas);
		
	    if(stFltGblSetting.bFlt1DotOn)
	    	Flt1Pt(vcPoint,vcPointData,dAngStLast, dAngEndLast, stFltLidarCfg.nSpinSpeed,stFltLidarCfg.nFPS,fFacAdj); 		

		if(nFltRobustChk)
		{	
			if(stFltGblSetting.bFlt2DotOn)			
				Flt2Pt(vcPoint,vcPointData,dAngStLast, dAngEndLast, stFltLidarCfg.nSpinSpeed,stFltLidarCfg.nFPS,fFacAdj,bEnAngLmt); 	 
			
			if(stFltGblSetting.nFltNDotOn > 2)
				FltNPt(vcPoint,vcPointData,dAngStLast, dAngEndLast, stFltLidarCfg.nSpinSpeed,stFltLidarCfg.nFPS,fFacAdj,stFltGblSetting.nFltNDotOn,bEnAngLmt);
		}	
	}
	else if(eFltType == FLT_NORMAL) //Normal-Flt 
	{   	    
		if(stFltGblSetting.bFltRayLOn)
			FltL(vcPoint,vcPointData,0, 0, stFltLParas);
		
		if(stFltGblSetting.bFlt1DotOn)
			Flt1Pt(vcPoint,vcPointData,0, 0, stFltLidarCfg.nSpinSpeed,stFltLidarCfg.nFPS,fFacAdj);		
		
		if(stFltGblSetting.bFlt2DotOn)			
			Flt2Pt(vcPoint,vcPointData,0, 0, stFltLidarCfg.nSpinSpeed,stFltLidarCfg.nFPS,fFacAdj,false);	 
			
		if(stFltGblSetting.nFltNDotOn > 2)
			FltNPt(vcPoint,vcPointData,0, 0, stFltLidarCfg.nSpinSpeed,stFltLidarCfg.nFPS,fFacAdj,stFltGblSetting.nFltNDotOn,false);
	}	
}

//**************************************************************************************************************************/
// 常景下检测滤除函数
// 输入: 
//       lstPointCloud - 1 圈点云数据
//       stFltLidarCfg - Paras of 雷达  
//       stFltGblSetting - Global 参数
//       stFltLParas - Line filter 参数
//输出:
//       lstPointCloud - 处理后的点云数据
//**************************************************************************************************************************/
void HCLidarFilter::hotPixelCheck(lstPtCloud_t &lstPointCloud,const stFltLidarCfg_t &stFltLidarCfg,stFltGblSetting_t stFltGblSetting,stFltLParas_t stFltLParas,eFltType_t eFltType,double dAngSt, double dAngEnd)
{
	bool bHSpd = false;
	float fFacAdj = stFltGblSetting.fFacAdjHSpd;	
    if(nLastSpd==0)   
    	nLastSpd =  stFltLidarCfg.nSpinSpeed;
    else
    {      
		if(nLastSpd > stFltLidarCfg.nLHSpdThres)  
			
		{
			bHSpd = true;
			fFacAdj = stFltGblSetting.fFacAdjHSpd;       
		}
		else 
		{
			bHSpd = false;
			fFacAdj = stFltGblSetting.fFacAdjLSpd;
		}		
		nLastSpd = stFltLidarCfg.nSpinSpeed;
	}

	vector<stFltCloudD_t> vcPointData;
	vcPointData.clear();
	for (int i = 0; i < lstPointCloud.size(); i++)
	{
		if (lstPointCloud[i].u16Dist && lstPointCloud[i].u16Dist <= stFltGblSetting.nActDist)
		{   
		    if(stFltGblSetting.dActScopeSt == stFltGblSetting.dActScopeEnd
               ||((stFltGblSetting.dActScopeSt != stFltGblSetting.dActScopeEnd)
				&& (stFltGblSetting.bScopeSmallIs && angleInRange(lstPointCloud[i].dAngle, stFltGblSetting.dActScopeSt, stFltGblSetting.dActScopeEnd) 
				|| stFltGblSetting.bScopeSmallIs==false && angleInRange(lstPointCloud[i].dAngle, stFltGblSetting.dActScopeSt, stFltGblSetting.dActScopeEnd)==false)))
		    {		
				stFltCloudD_t pTemp;
				double angle_rad_cur = lstPointCloud[i].dAngle * CV_PI / 180;
				unsigned short u16Dist = lstPointCloud[i].u16Dist;
				pTemp.x = u16Dist * cos(angle_rad_cur);
				pTemp.y = u16Dist * sin(angle_rad_cur);
				pTemp.u16Dist = u16Dist;
				pTemp.dAngle = lstPointCloud[i].dAngle;
				pTemp.nfound = 0;
				vcPointData.push_back(pTemp);
			}
		}					
	}    		  
	sort(vcPointData.begin(), vcPointData.end(), comP2DAng);   

	lstPtCloud_t &vcPoint = lstPointCloud; 
	if(stFltGblSetting.bFltRayLOn)
		FltL(vcPoint,vcPointData,0, 0, stFltLParas);
	
    if(stFltGblSetting.bFlt1DotOn)
    	Flt1Pt(vcPoint,vcPointData,0, 0, stFltLidarCfg.nSpinSpeed,stFltLidarCfg.nFPS,fFacAdj); 		

	if(stFltGblSetting.bFlt2DotOn)			
		Flt2Pt(vcPoint,vcPointData,0, 0, stFltLidarCfg.nSpinSpeed,stFltLidarCfg.nFPS,fFacAdj,false); 	 
		
	if(stFltGblSetting.nFltNDotOn > 2)
		FltNPt(vcPoint,vcPointData,0, 0, stFltLidarCfg.nSpinSpeed,stFltLidarCfg.nFPS,fFacAdj,stFltGblSetting.nFltNDotOn,false);	  
}

/**************************************************************************************************************************/
// Ray line 滤除
// 输入: 
//       vcPoint - 1圈点云数据
//       vcPointData - 作用域内 X/Y 坐标点云
//       dAngSt  - No use，预留
//       dAngEnd - No use, 预留
// 输出:
//       vcPoint - 处理后的点云数据
//       vcPointData - 作用域内 X/Y 坐标点云
/**************************************************************************************************************************/
void HCLidarFilter::FltL(lstPtCloud_t &vcPoint,vector<stFltCloudD_t> &vcPointData,const double &dAngSt,const double &dAngEnd,stFltLParas_t stFltLParas)
{    	
    if(vcPointData.size()==0) return;
	
	vector<stFltCloudD_t> &vcDistPoint = vcPointData;
	vector<stFltCloudD_t> veIndex;
	vector<stFltCloudD_t> veIndexArr;
	veIndexArr.clear();
	veIndex.clear();
	vector<int> veRcdIdx;
	veRcdIdx.clear();
	bool bNormal = true;
	double dASt=0,dAEnd=0;
	int i, j, n;
	for (i = 0; i < vcDistPoint.size(); i++)
	{
		if (vcDistPoint[i].u16Dist > stFltLParas.nActDist)
			continue;

		if (vcDistPoint[i].nfound == 0x5aa5) 
				continue;

        dASt = vcDistPoint[i].dAngle - 25;
		dAEnd = vcDistPoint[i].dAngle + 25;
		if (dAEnd >= 360) 
		{
			dAEnd = dAEnd - 360;
		}
		if (dASt < 0)
		{
			dASt = 360 + dASt;
		}
        if(dASt > dAEnd)
        {
			double dTemp = dASt;
			dASt =dAEnd;
			dAEnd = dTemp;
		}
				        
		double x1 = vcDistPoint[i].x;
		double y1 = vcDistPoint[i].y;
		 
		for (j = 0; j < vcDistPoint.size(); j++)	
		{
			if(j==i)
				continue;
		     
			if (vcDistPoint[j].u16Dist <= stFltLParas.nActDist)  
			{ 
				//if (vcDistPoint[j].nfound == 0x5aa5) 
				//	continue;

				if(angleInRange(vcDistPoint[j].dAngle ,dASt,dAEnd)==false)
					continue;

				double x2 = vcDistPoint[j].x;
				double y2 = vcDistPoint[j].y;
				double k = (x1 - x2) ? (y1 - y2) / (x1 - x2) : 0;

				double b = y1 - k * x1;
				//double r = k ? (-b / k) : 10000;

				double dtoorg = (b)/sqrt(k*k+1);				
				if (fabs(dtoorg) > stFltLParas.nCirCenFld)
					continue;
				
				int nCountFnd = 0;
				for (n = 0; n < vcDistPoint.size(); n++)
				{
					if (vcDistPoint[n].u16Dist > stFltLParas.nActDist)
						continue;

                    if(n==i || n==j)
						continue;
											
					//if (vcDistPoint[n].nfound == 0x5aa5)
					//	continue;
					
					if(angleInRange(vcDistPoint[n].dAngle ,dASt,dAEnd)==false)
						continue;

					double x3 = vcDistPoint[n].x;
					double y3 = vcDistPoint[n].y;					
                    double yc3 = (k*x3-y3+b)/sqrt(k*k+1);			
					if (fabs(yc3) <= stFltLParas.nPtToLineMin)
					{
						nCountFnd++;
						veIndex.push_back(vcDistPoint[n]);
						veRcdIdx.push_back(n);
					}
				}
                
				if (nCountFnd >= stFltLParas.nPtOfLineMin ) //|| vcPointData.size()<= (stFltLParas.nPtOfLineMin+2)&& stFltLParas.nPtOfLineMin)  
				{						
                    int f;
					int nok=1;
					sort(veIndex.begin(),veIndex.end(),comP2DDist);
					int dst0 = veIndex[0].u16Dist;
					double angst0 = veIndex[0].dAngle;					
					for (f = 1; f < veIndex.size(); f++)
					{					     
                         if(abs(int(veIndex[f].u16Dist - dst0))>=3 || fabs(veIndex[f].dAngle - angst0)>=2.4 
						 	|| vcPointData.size() <= (stFltLParas.nPtOfLineMin +2))
                         {
						 	nok++;
							dst0 = veIndex[f].u16Dist;
							angst0 =veIndex[f].dAngle;
                         }	
						 if(nok>= stFltLParas.nPtOfLineMin)
						 	break;
					}
					if(nok>= stFltLParas.nPtOfLineMin) 
					{				
						for (f = 0; f < veIndex.size(); f++)
						{
							veIndexArr.push_back(veIndex[f]);
						}
						veIndexArr.push_back(vcDistPoint[i]);
						veIndexArr.push_back(vcDistPoint[j]);

						for (int s = 0; s < veRcdIdx.size(); s++)
						{
							vcDistPoint[veRcdIdx[s]].nfound = 0x5aa5;
						}
						vcDistPoint[i].nfound = 0x5aa5;  
						vcDistPoint[j].nfound = 0x5aa5;
					}

				}
				veIndex.clear();
				veRcdIdx.clear();
									
			}

		}

	}

	
	if (veIndexArr.size())
	{
		for (int i = 0; i < veIndexArr.size(); i++)
		{
			for (int j = 0; j < vcPoint.size(); j++)
			{
				if (veIndexArr[i].dAngle == vcPoint[j].dAngle && veIndexArr[i].u16Dist == vcPoint[j].u16Dist)
				{
					vcPoint.erase(vcPoint.begin()+j);
					j = j - 1;
					break;
				}					
			}
		}
	}			
	#if EN_INHERIT==1
	for (int k = 0; k < vcDistPoint.size(); k++)
	{
		if (vcDistPoint[k].nfound == 0x5aa5)
		{
			vcDistPoint.erase(vcDistPoint.begin()+k);
			k = k - 1;
		}
	}
	#endif			
}


/**************************************************************************************************************************/
// 单点 滤除
// 输入: 
//       vcPoint - 1圈点云数据
//       vcPointData - 作用域内 X/Y 坐标点云
//       dAngSt  - No use, 预留
//       dAngEnd - No use, 预留
//       nSpinSpeed - RPM
//       giFilterFPS - FPS
//       fFacAdj - (0-6)均衡系数
// 输出:
//       vcPoint - 结果点云数据
//       vcPointData - 处理后的作用域内 X/Y 坐标点云
/**************************************************************************************************************************/

void HCLidarFilter::Flt1Pt(lstPtCloud_t &vcPoint,vector<stFltCloudD_t> &vcPointData,const double &dAngSt,const double &dAngEnd, int giFilterSpeed,int giFilterFPS,float fFacAdj)
{    
    if(vcPointData.size()==0) return;
	double dAngIsolution = giFilterSpeed /60 * 2* CV_PI / giFilterFPS;
	double dDistOfPtFac  = 2*sin(dAngIsolution/2); 
		
	#ifdef EN_THRES_CALIBRTE
		double dCaliFac = dAngIsolution/0.010472;  
		dCaliFac = dCaliFac*dCaliFac;
		dCaliFac =dCaliFac*fFacAdj; 
	#else //Calculate
	#define  SINGLE_PT_FAC sqrt(2)*sqrt(2)		
	double  dCaliFac = fFacAdj;
	#endif	

    double dThresSet = 8; 
	std::vector<stFltCloudD_t> &vcPoint1Alias = vcPointData;						
    int j=0,i=0;
	
	for(i = 0; i < vcPoint1Alias.size(); i++)
	{			  
		double x1 = vcPoint1Alias[i].x;
		double y1 = vcPoint1Alias[i].y;
		
		unsigned int u16Dist = vcPoint1Alias[i].u16Dist;
		double dAngle = vcPoint1Alias[i].dAngle;
		 
		GetDistDotThres(vcPoint1Alias[i].u16Dist,dDistOfPtFac,dCaliFac,dThresSet);
       
		for(j = i+1; j < vcPoint1Alias.size(); j++)
		{						   
			double x2 = vcPoint1Alias[j].x;
			double y2 = vcPoint1Alias[j].y;
			//if(u16Dist ==  vcPoint1Alias[j].u16Dist  && dAngle == vcPoint1Alias[j].dAngle)  //same pt or not 
			//	continue;
			
			if(((x1-x2)*(x1-x2)+ (y1-y2)*(y1-y2))< dThresSet)//relate to spin speed
			{			    
				vcPoint1Alias[i].nfound = 0x5757; //is not
				vcPoint1Alias[j].nfound = 0x5757; //is not				 
			}
		}
	}

	for(i = 0; i < vcPoint1Alias.size(); i++) 
	{
	    if(vcPoint1Alias[i].nfound == 0x5757)
			continue;			
		for(j = 0; j < vcPoint.size(); j++)
		{			
			if(vcPoint1Alias[i].dAngle == vcPoint[j].dAngle && vcPoint1Alias[i].u16Dist == vcPoint[j].u16Dist)
			{
				vcPoint.erase(vcPoint.begin()+j);
				j=j-1;				
				break;
			}			
		}
    }
	
	#if EN_INHERIT==1
	for(i = 0; i < vcPoint1Alias.size(); i++)
	{
		if(vcPoint1Alias[i].nfound != 0x5757)
		{ 				   
			vcPoint1Alias.erase(vcPoint1Alias.begin()+i);
			i=i-1;			
		} 		  
	}   
	#endif	  		
}

/**************************************************************************************************************************/
// 双点 滤除
// 输入: 
//       vcPoint - 1圈点云数据
//       vcPointData - 作用域内 X/Y 坐标点云
//       dAngSt  - 起始角度限定
//       dAngEnd - 结束角度限定
//       nSpinSpeed - RPM
//       giFilterFPS - FPS
//       fFacAdj - (0-6)均衡系数
//       bEnAngLmt - 角度限定使能标记  
// 输出:
//       vcPoint - 结果点云数据
//       vcPointData - 处理后的作用域内 X/Y 坐标点云
/**************************************************************************************************************************/

void HCLidarFilter::Flt2Pt(lstPtCloud_t &vcPoint,vector<stFltCloudD_t> &vcPointData,const double &dAngSt,const double &dAngEnd, int giFilterSpeed,int giFilterFPS,
                             float fFacAdj,bool bEnAngLmt)
{			    
    if(vcPointData.size()==0) return;
	double dAngIsolution = giFilterSpeed / 60 * 2 * CV_PI / giFilterFPS;
	double dDistOfPtFac = 2 * sin(dAngIsolution / 2);

#ifdef EN_THRES_CALIBRTE
	double dCaliFac = dAngIsolution / 0.010472;		
	dCaliFac = dCaliFac * dCaliFac;
	dCaliFac =dCaliFac*fFacAdj;

#else 
	#define  SINGLE_PT_FAC sqrt(2)*sqrt(2)		
	double	dCaliFac = fFacAdj;

#endif  

	vector<stFltCloudD_t> vcPointAlias(vcPointData);
	int j = 0;
	 bool bNormal = true ;
	 if(dAngSt <= 180 && fabs(360-dAngEnd + dAngSt)<181)		
		bNormal = false;

	vector<stFltCloudD_t> vcPointGet;
	vcPointGet.clear();

	double  dThresSet = 8;
	for (int i = 0; i < vcPointAlias.size(); i++)
	{
	//#if EN_ANG_LIM
		if(bEnAngLmt)
		{
			if(bNormal)	
			{
		     	if (vcPointAlias[i].dAngle < dAngSt || vcPointAlias[i].dAngle > dAngEnd)
					continue;						
			}
			else
			{
				if (vcPointAlias[i].dAngle > dAngSt && vcPointAlias[i].dAngle < dAngEnd)
					continue;							
			}
		}
	//#endif	
										     
		double x1 = vcPointAlias[i].x;
		double y1 = vcPointAlias[i].y;

		GetDistDotThres(vcPointAlias[i].u16Dist,dDistOfPtFac,dCaliFac,dThresSet);
				
		bool b1Pt = true;
		for (j = 0; j < vcPointAlias.size(); j++)
		{
            //#if EN_ANG_LIM
            if(bEnAngLmt)
            {
				if(bNormal)	
				{
		         	if (vcPointAlias[j].dAngle < dAngSt || vcPointAlias[j].dAngle > dAngEnd)
						continue;						
				}
				else
				{
					if (vcPointAlias[j].dAngle > dAngSt && vcPointAlias[j].dAngle < dAngEnd)
						continue;							
				}
            }
			//#endif
			
			double x2 = vcPointAlias[j].x;
			double y2 = vcPointAlias[j].y;
			if (vcPointAlias[i].u16Dist == vcPointAlias[j].u16Dist  && vcPointAlias[i].dAngle == vcPointAlias[j].dAngle)	  // 
				continue;

			bool b2Pt = true;
			if (((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2)) < dThresSet)
			{
				b1Pt = false;
				for (int n = 0; n < vcPointAlias.size(); n++)
				{
					if (vcPointAlias[n].u16Dist == vcPointAlias[i].u16Dist  && vcPointAlias[n].dAngle == vcPointAlias[i].dAngle
						|| vcPointAlias[n].u16Dist == vcPointAlias[j].u16Dist  && vcPointAlias[n].dAngle == vcPointAlias[j].dAngle)	  // 
						continue;
                    //#if EN_ANG_LIM
                    if(bEnAngLmt)
                    {
						if(bNormal)	
						{
		                 	if (vcPointAlias[n].dAngle < dAngSt || vcPointAlias[n].dAngle > dAngEnd)
								continue;						
						}
						else
						{
							if (vcPointAlias[n].dAngle > dAngSt && vcPointAlias[n].dAngle < dAngEnd)
								continue;							
						}
                    }
					//#endif

					double x3 = vcPointAlias[n].x;
					double y3 = vcPointAlias[n].y;

					if (((x3 - x1)*(x3 - x1) + (y3 - y1)*(y3 - y1)) < dThresSet
						|| ((x3 - x2)*(x3 - x2) + (y3 - y2)*(y3 - y2)) < dThresSet)
					{
						b2Pt = false;
						break;
					}
				}
			}

			if (b1Pt == false && b2Pt)	
			{
				vcPointGet.push_back(vcPointAlias[i]);
				vcPointGet.push_back(vcPointAlias[j]);
				break;	 				 
			}
			else if (b1Pt)	 
			{
				continue;
			}

			else if (b1Pt == false && b2Pt == false)  
			{
				break; 
			}
		}

		if (b1Pt )  
		{
			vcPointGet.push_back(vcPointAlias[i]);
		}	

	}

	for (int i = 0; i < vcPointGet.size(); i++)
	{					
		for (int j = 0; j < vcPoint.size(); j++)
		{
			if (vcPointGet[i].dAngle == vcPoint[j].dAngle && vcPointGet[i].u16Dist == vcPoint[j].u16Dist)
			{
				vcPoint.erase(vcPoint.begin()+j);
				break;
			}
		}
#if EN_INHERIT==1
		for(int k = 0; k < vcPointData.size(); k++)
		{
			if(vcPointGet[i].dAngle == vcPointData[k].dAngle && vcPointGet[i].u16Dist == vcPointData[k].u16Dist)
			{					 
				vcPointData.erase(vcPointData.begin()+k);
				break;
			}			
		}	
#endif
		
	}
}


/**************************************************************************************************************************/
// 3+点 滤除
// 输入: 
//       vcPoint - 1圈点云数据
//       vcPointData - 作用域内 X/Y 坐标点云
//       dAngSt  - 起始角度限定
//       dAngEnd - 结束角度限定
//       nSpinSpeed - RPM
//       giFilterFPS - FPS
//       fFacAdj - (0-6)均衡系数
//       nTrace - 滤除点数设定
//       bEnAngLmt - 角度限定使能标记  
// 输出:
//       vcPoint - 结果点云数据
//       vcPointData - 处理后的作用域内 X/Y 坐标点云
/**************************************************************************************************************************/
void HCLidarFilter::FltNPt(lstPtCloud_t &vcPoint,vector<stFltCloudD_t> &vcPointData,const double &dAngSt,const double &dAngEnd, int giFilterSpeed,
                             int giFilterFPS,float fFacAdj,int nTrace,bool bEnAngLmt)
{				
	double dAngIsolution = giFilterSpeed / 60 * 2 * CV_PI / giFilterFPS;
	double dDistOfPtFac = 2 * sin(dAngIsolution / 2);
	
#ifdef EN_THRES_CALIBRTE
	double dCaliFac = dAngIsolution / 0.010472;		
	dCaliFac = dCaliFac * dCaliFac;
	dCaliFac =dCaliFac*fFacAdj;

#else 
	#define  SINGLE_PT_FAC sqrt(2)*sqrt(2)		 
	double	dCaliFac = fFacAdj;

#endif  

	vector<stFltCloudD_t> vcPointAlias(vcPointData);
	int j = 0,i=0,k=0;
	 bool bNormal = true ;
	 if(dAngSt <= 180 && fabs(360-dAngEnd + dAngSt)<181)		
		bNormal = false;

	vector<stFltCloudD_t> vcPointGet;
	vcPointGet.clear();
	double  dThresSet = 8;


    bool bSeek = false;
    int is=0,ie =0;	
	for ( i = 0; i < vcPointAlias.size(); i++)
	{
	//#if EN_ANG_LIM
		if(bEnAngLmt)
		{
	
			if(bNormal)	
			{
		     	if (vcPointAlias[i].dAngle < dAngSt || vcPointAlias[i].dAngle > dAngEnd)
					continue;						
			}
			else
			{
				if (vcPointAlias[i].dAngle > dAngSt && vcPointAlias[i].dAngle < dAngEnd)
					continue;							
			}
		}	
	//#endif	
										     
		double x1 = vcPointAlias[i].x;
		double y1 = vcPointAlias[i].y;
		
		double x01 = x1;
		double y01 = y1;

		GetDistDotThres(vcPointAlias[i].u16Dist,dDistOfPtFac,dCaliFac,dThresSet);

		bSeek = false;
        is = i;	
		ie = i;
		
		for (j = i; j < vcPointAlias.size(); j++)
		{
	//#if EN_ANG_LIM
	        if(bEnAngLmt)
	        {
				if(bNormal)	
				{
		         	if (vcPointAlias[j].dAngle < dAngSt || vcPointAlias[j].dAngle > dAngEnd)
						continue;						
				}
				else
				{
					if (vcPointAlias[j].dAngle > dAngSt && vcPointAlias[j].dAngle < dAngEnd)
						continue;							
				}
	        }
	//#endif			
			double x2 = vcPointAlias[j].x;
			double y2 = vcPointAlias[j].y;
			
			if (vcPointAlias[i].u16Dist == vcPointAlias[j].u16Dist  && vcPointAlias[i].dAngle == vcPointAlias[j].dAngle)  //seems go for nothing	   
				continue;				

			if (((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2)) < dThresSet  || ((x01 - x2)*(x01 - x2) + (y01 - y2)*(y01 - y2)) < dThresSet)
			{			
				ie  = j;				
				x01 = x1;
				y01 = y1;
				
				x1 = x2;
		        y1 = y2;				
				continue;  //to seek next one
			}          					
            if((j-is) >= nTrace * 2) //total  condition
            {
                bSeek = true; 
				break;
            }
		}
		       
        bool bI0BlkIs = false;  //0 index flag 
        if(i==0 || (ie+nTrace*2) > vcPointAlias.size())
        {
			if((ie-is)>nTrace)
				bI0BlkIs = true;	
		} 
											  
		if(i==0 && bI0BlkIs==false && vcPointAlias.size() > nTrace )  //backtrace 
		{		     
		        int is1=vcPointAlias.size()-1,ie1=vcPointAlias.size()-1;
            
			    k = vcPointAlias.size()-1;
            //#if EN_ANG_LIM
				if(bEnAngLmt)
				{            
					if(bNormal) 
					{
						if (vcPointAlias[k].dAngle < dAngSt || vcPointAlias[k].dAngle > dAngEnd)
							continue;						
					}
					else
					{
						if (vcPointAlias[k].dAngle > dAngSt && vcPointAlias[k].dAngle < dAngEnd)
							continue;							
					}
				}	
            //#endif	
				x1 = vcPointAlias[k].x;
				y1 = vcPointAlias[k].y;
				
				x01 = vcPointAlias[0].x;
				y01 = vcPointAlias[0].y;
						
				bSeek = false;
				is1 = k; 
				ie1 = k;
				
				for (j = k; j >=ie; j--)
				{
                //#if EN_ANG_LIM
					if(bEnAngLmt)
					{                
						if(bNormal) 
						{
							if (vcPointAlias[j].dAngle < dAngSt || vcPointAlias[j].dAngle > dAngEnd)
								continue;						
						}
						else
						{
							if (vcPointAlias[j].dAngle > dAngSt && vcPointAlias[j].dAngle < dAngEnd)
								continue;							
						}
					}	
               //#endif			
					double x2 = vcPointAlias[j].x;
					double y2 = vcPointAlias[j].y;
					
					if (vcPointAlias[k].u16Dist == vcPointAlias[j].u16Dist  && vcPointAlias[k].dAngle == vcPointAlias[j].dAngle)  //seems go for nothing	   
						continue;				
			
					if (((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2)) < dThresSet  || ((x01 - x2)*(x01 - x2) + (y01 - y2)*(y01 - y2)) < dThresSet)
					{			
						ie1	= j;				
						x01 = x1;
						y01 = y1;
						
						x1 = x2;
						y1 = y2;				
						continue;  //to seek next one
					}
			
					if((is1 - j) >= nTrace * 2) //total  condition
					{
						bSeek = true; 
						break;
					}
				}
			
				//if(ie1 != is1)  
				{				
                    for (k = ie1; k <= is1 ; k++)
					{
					    vcPointGet.push_back(vcPointAlias[k]);						
					}
				}				            
		}
        
		else if((ie+nTrace*2) > vcPointAlias.size() && bI0BlkIs==false && vcPointAlias.size() > nTrace )   
		{
			int is1=0,ie1=0;
		
			k = 0;
		//#if EN_ANG_LIM
	        if(bEnAngLmt)
	        {
		
				if(bNormal) 
				{
					if (vcPointAlias[k].dAngle < dAngSt || vcPointAlias[k].dAngle > dAngEnd)
						continue;						
				}
				else
				{
					if (vcPointAlias[k].dAngle > dAngSt && vcPointAlias[k].dAngle < dAngEnd)
						continue;							
				}
	        }	
		//#endif	
		
			x1 = vcPointAlias[k].x;
			y1 = vcPointAlias[k].y;
			
			//x01 = vcPointAlias[0].x;
			//y01 = vcPointAlias[0].y;			
			
			is1 = k; 
			ie1 = k;			
			for (j = k+1; j <vcPointAlias.size() ; j++)
			{
			//#if EN_ANG_LIM
				if(bEnAngLmt)
				{			
					if(bNormal) 
					{
						if (vcPointAlias[j].dAngle < dAngSt || vcPointAlias[j].dAngle > dAngEnd)
							continue;						
					}
					else
					{
						if (vcPointAlias[j].dAngle > dAngSt && vcPointAlias[j].dAngle < dAngEnd)
							continue;							
					}
				}	
   			//#endif			
				double x2 = vcPointAlias[j].x;
				double y2 = vcPointAlias[j].y;
				
				if (vcPointAlias[k].u16Dist == vcPointAlias[j].u16Dist  && vcPointAlias[k].dAngle == vcPointAlias[j].dAngle)  //seems go for nothing	   
					continue;				
		
				if (((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2)) < dThresSet  || ((x01 - x2)*(x01 - x2) + (y01 - y2)*(y01 - y2)) < dThresSet)
				{			
					ie1 = j;				
					x01 = x1;
					y01 = y1;
					
					x1 = x2;
					y1 = y2;				
					continue;  //to seek next one
				}
		
				if((is1 - j) >= nTrace * 2) 
				{
					bSeek = true; 
					break;
				}
			}
									
			for (k = is1; k <= ie1 ; k++)
			{
				vcPointGet.push_back(vcPointAlias[k]);						
			}			
		}

		for(k = is; k <= ie ; k++)
		{
			vcPointGet.push_back(vcPointAlias[k]);						
		}
		i = ie+1;
	}

	for ( i = 0; i < vcPointGet.size(); i++)
	{					
		for ( j = 0; j < vcPoint.size(); j++)
		{
			if (vcPointGet[i].dAngle == vcPoint[j].dAngle && vcPointGet[i].u16Dist == vcPoint[j].u16Dist)
			{
				vcPoint.erase(vcPoint.begin()+j);
				break;
			}
		}
#if EN_INHERIT==1
		for( k = 0; k < vcPointData.size(); k++)
		{
			if(vcPointGet[i].dAngle == vcPointData[k].dAngle && vcPointGet[i].u16Dist == vcPointData[k].u16Dist)
			{					 
				vcPointData.erase(vcPointData.begin()+k);
				break;
			}			
		}	
#endif
		
	}

}


/**************************************************************************************************************************/
// Dot - Dot: Distance 
/**************************************************************************************************************************/

#define SINGLE_DIST_OFST 8
void HCLidarFilter::GetDistDotThres(unsigned int nDist,double &dDistOfPtFac, double &dCaliFac, double &dThresSet)
{

#ifdef EN_THRES_CALIBRTE
	if (nDist)  // 
	{
		if (nDist < 200)
			dThresSet = 10;
		else if (nDist < 225)
			dThresSet = 10 + SINGLE_DIST_OFST / 4;	 //C=  S=12
		else if (nDist < 250)
			dThresSet = 10 + SINGLE_DIST_OFST / 2;	 //C=  S=14 		
		else if (nDist < 275)
			dThresSet = 10 + SINGLE_DIST_OFST * 3 / 4;	//C=  S=16
		else if (nDist < 300)
			dThresSet = 10 + SINGLE_DIST_OFST;	  //C=	S=18			
		else if (nDist < 350)
			dThresSet = 10 + SINGLE_DIST_OFST + SINGLE_DIST_OFST / 2;  //C=  S=22				
		else if (nDist < 400)
			dThresSet = 10 + 3 * SINGLE_DIST_OFST;						 //C=  S=34 	
		else if (nDist < 500)
			dThresSet = 10 + 6 * SINGLE_DIST_OFST;	//C=118    S=58
		else if (nDist < 600)
			dThresSet = 10 + 10 * SINGLE_DIST_OFST; //C=118    S=90 	
		else if (nDist < 700)
			dThresSet = 10 + 14 * SINGLE_DIST_OFST; //C=118    S=122
		else if (nDist < 800)
			dThresSet = 10 + 18 * SINGLE_DIST_OFST; //C=118    S=154		
		else if (nDist < 900)
			dThresSet = 10 + 22 * SINGLE_DIST_OFST; //C=150    S=186
		else if (nDist <= 1000)
			dThresSet = 10 + 25 * SINGLE_DIST_OFST; //C=185.23 S=210		
	}

	dThresSet = dThresSet * (dCaliFac);

#else
	if (nDist)
	{
		dThresSet = nDist * dDistOfPtFac;
		dThresSet = dThresSet * dThresSet;
		dThresSet = dThresSet * SINGLE_PT_FAC;
		dThresSet = dThresSet * (dCaliFac);
	}
#endif

}

/**************************************************************************************************************************/
//condition Comment: dAngSt<=dAngEnd
/**************************************************************************************************************************/
bool HCLidarFilter::angleInRange(const double dAngleIn, const double &dAngSt , const double &dAngEnd )
{
	bool bAbnormal = false ;
	if(dAngSt <= 180 && fabs(360-dAngEnd + dAngSt)<181) 	   
	   bAbnormal = true;

	bool bInAngle = false ;			
	if(bAbnormal) 
	{
	   if (dAngleIn < dAngSt || dAngleIn > dAngEnd)
		   bInAngle = true; 				   
	}
	else
	{
	   if (dAngleIn > dAngSt && dAngleIn < dAngEnd)
		   bInAngle = true ;						   
	}	
	return bInAngle;
}

/**************************************************************************************************************************/
// 输入:  
//      sLidarModel-eg.  "X2F" 
//      bLowSpeed - true for Low Spin-Speed
// 输出:
//      stFltLidarCfg - Lidar Paras 
// Return:
//      true: sLidarModel exisit in list,false: not in list
/**************************************************************************************************************************/
bool HCLidarFilter::UpdateLidarPara(const std::string         sLidarModel, bool bLowSpeed, stFltLidarCfg_t &stFltLidarCfg)
{
    if(sLidarModel == X1B)
    {    
	    stFltLidarCfg.nBlindDist = BLND_DIST_X1;
		stFltLidarCfg.nFPS = FPS_2000_NOR;
		stFltLidarCfg.nSpinSpeed = SPEED_312_NOR;
		stFltLidarCfg.nLHSpdThres =  SPEED_312_NOR - 50; 		
    }
    else if(sLidarModel == X1D)
    {
		stFltLidarCfg.nBlindDist = BLND_DIST_X1;
		stFltLidarCfg.nFPS = FPS_2000_NOR;
		stFltLidarCfg.nSpinSpeed = SPEED_312_NOR;
		stFltLidarCfg.nLHSpdThres =  SPEED_312_NOR - 50;		
    }
    else if(sLidarModel == X1E)
    {
		stFltLidarCfg.nBlindDist = BLND_DIST_X1;
		stFltLidarCfg.nFPS = FPS_2000_NOR;
		stFltLidarCfg.nSpinSpeed = SPEED_312_NOR;
		stFltLidarCfg.nLHSpdThres =  SPEED_312_NOR - 50;		
    }
    else if(sLidarModel == X1F)
    {
		stFltLidarCfg.nBlindDist = BLND_DIST_X1;
		stFltLidarCfg.nFPS = FPS_2000_NOR;
		stFltLidarCfg.nSpinSpeed = SPEED_312_NOR;
		stFltLidarCfg.nLHSpdThres =  SPEED_312_NOR - 50;	

    }
	else if (sLidarModel == X1G)
	{
		stFltLidarCfg.nBlindDist = BLND_DIST_X1;
		stFltLidarCfg.nFPS = FPS_3000_NOR;
		stFltLidarCfg.nSpinSpeed = SPEED_312_NOR;
		stFltLidarCfg.nLHSpdThres =  SPEED_312_NOR - 50;

	}
	else if (sLidarModel == X1K)
	{
		stFltLidarCfg.nBlindDist = BLND_DIST_X1;
		stFltLidarCfg.nFPS = FPS_2000_NOR;
		stFltLidarCfg.nSpinSpeed = SPEED_312_NOR;
		stFltLidarCfg.nLHSpdThres =  SPEED_312_NOR - 50;
	}
	else if (sLidarModel == X1L)
	{
		stFltLidarCfg.nBlindDist = BLND_DIST_X1;
		stFltLidarCfg.nFPS = FPS_2000_NOR;
		stFltLidarCfg.nSpinSpeed = SPEED_312_NOR;
		stFltLidarCfg.nLHSpdThres =  SPEED_312_NOR - 50;
	}
    else if(sLidarModel == X1M)
    {
		stFltLidarCfg.nBlindDist = BLND_DIST_X1;
		stFltLidarCfg.nFPS = FPS_2000_NOR;
		stFltLidarCfg.nSpinSpeed = SPEED_312_NOR;
		stFltLidarCfg.nLHSpdThres =  SPEED_312_NOR - 50;
    }
	else if (sLidarModel == X1N)
	{
		stFltLidarCfg.nBlindDist = BLND_DIST_X1;
		stFltLidarCfg.nFPS = FPS_3000_NOR;
		stFltLidarCfg.nSpinSpeed = SPEED_312_NOR;
		stFltLidarCfg.nLHSpdThres =  SPEED_312_NOR - 50;
	}
	else if (sLidarModel == X1S)
	{
		stFltLidarCfg.nBlindDist = BLND_DIST_X1;
		stFltLidarCfg.nFPS = FPS_1800_NOR;
		stFltLidarCfg.nSpinSpeed = SPEED_300_NOR;
		stFltLidarCfg.nLHSpdThres =  SPEED_300_NOR - 50;
	}
    else if(sLidarModel == X2A)
    {
		stFltLidarCfg.nBlindDist = BLND_DIST_X2;
		stFltLidarCfg.nFPS = FPS_3000_NOR;
		stFltLidarCfg.nSpinSpeed = SPEED_360_NOR;
		stFltLidarCfg.nLHSpdThres =  SPEED_360_NOR - 50;
    }
    else if(sLidarModel == X2B)
    {
		stFltLidarCfg.nBlindDist = BLND_DIST_X2;
		stFltLidarCfg.nFPS = FPS_2000_NOR;
		stFltLidarCfg.nSpinSpeed = SPEED_312_NOR;
		stFltLidarCfg.nLHSpdThres =  SPEED_312_NOR - 50;		
	}
    else if(sLidarModel == X2BF)
    {
        if(bLowSpeed)
        {
			stFltLidarCfg.nBlindDist = BLND_DIST_X2;
			stFltLidarCfg.nFPS = FPS_3000_NOR;
			stFltLidarCfg.nSpinSpeed = SPEED_180_NOR;
			stFltLidarCfg.nLHSpdThres =  (SPEED_180_NOR + SPEED_360_NOR)/2;		
        }
		else
        {
			stFltLidarCfg.nBlindDist = BLND_DIST_X2;
			stFltLidarCfg.nFPS = FPS_3000_NOR;
			stFltLidarCfg.nSpinSpeed = SPEED_360_NOR;
			stFltLidarCfg.nLHSpdThres =  (SPEED_180_NOR + SPEED_360_NOR)/2;		
        }		
	}		
    else if(sLidarModel == X2C)
    {
		stFltLidarCfg.nBlindDist = BLND_DIST_X2;
		stFltLidarCfg.nFPS = FPS_3000_NOR;
		stFltLidarCfg.nSpinSpeed = SPEED_360_NOR;
		stFltLidarCfg.nLHSpdThres =  SPEED_360_NOR - 50;
    }
    else if(sLidarModel == X2E)
    {
		stFltLidarCfg.nBlindDist = BLND_DIST_X2;
		stFltLidarCfg.nFPS = FPS_3000_NOR;
		stFltLidarCfg.nSpinSpeed = SPEED_360_NOR;
		stFltLidarCfg.nLHSpdThres =  SPEED_360_NOR - 50;
    }
    else if(sLidarModel == X2F)
    {
		stFltLidarCfg.nBlindDist = BLND_DIST_X2;
		stFltLidarCfg.nFPS = FPS_3000_NOR;
		stFltLidarCfg.nSpinSpeed = SPEED_360_NOR;
		stFltLidarCfg.nLHSpdThres =  SPEED_360_NOR - 50;
    }
    else if(sLidarModel == X2M)
    {
		stFltLidarCfg.nBlindDist = BLND_DIST_X2;
		stFltLidarCfg.nFPS = FPS_2000_NOR;
		stFltLidarCfg.nSpinSpeed = SPEED_312_NOR;
		stFltLidarCfg.nLHSpdThres =  SPEED_312_NOR - 50;
    }
	else if(sLidarModel == X2MF)
	{
        if(bLowSpeed)
        {
			stFltLidarCfg.nBlindDist = BLND_DIST_X2;
			stFltLidarCfg.nFPS = FPS_2000_NOR;
			stFltLidarCfg.nSpinSpeed = SPEED_180_NOR;
			stFltLidarCfg.nLHSpdThres =  (SPEED_180_NOR + SPEED_360_NOR)/2;		
        }
		else
        {
			stFltLidarCfg.nBlindDist = BLND_DIST_X2;
			stFltLidarCfg.nFPS = FPS_2000_NOR;
			stFltLidarCfg.nSpinSpeed = SPEED_360_NOR;
			stFltLidarCfg.nLHSpdThres =  (SPEED_180_NOR + SPEED_360_NOR)/2;		
        }
	}	
    else if(sLidarModel == X2N)
    {
		stFltLidarCfg.nBlindDist = BLND_DIST_X2;
		stFltLidarCfg.nFPS = FPS_2000_NOR;
		stFltLidarCfg.nSpinSpeed = SPEED_312_NOR;
		stFltLidarCfg.nLHSpdThres =  SPEED_312_NOR - 50;		
    }
	else if (sLidarModel == X2YE)
	{
		stFltLidarCfg.nBlindDist = BLND_DIST_X2;
		stFltLidarCfg.nFPS = FPS_3000_NOR;
		stFltLidarCfg.nSpinSpeed = SPEED_315_NOR;
		stFltLidarCfg.nLHSpdThres =  SPEED_315_NOR - 50;		
	}
	else if (sLidarModel == X2YJ || sLidarModel == X2YP)
	{
        if(bLowSpeed)
        {
			stFltLidarCfg.nBlindDist = BLND_DIST_X2;
			stFltLidarCfg.nFPS = FPS_4000_NOR;
			stFltLidarCfg.nSpinSpeed = SPEED_330_NOR;
			stFltLidarCfg.nLHSpdThres =  (SPEED_330_NOR + SPEED_420_NOR)/2;
        }
		else
        {
			stFltLidarCfg.nBlindDist = BLND_DIST_X2;
			stFltLidarCfg.nFPS = FPS_4000_NOR;
			stFltLidarCfg.nSpinSpeed = SPEED_420_NOR;
			stFltLidarCfg.nLHSpdThres =  (SPEED_330_NOR + SPEED_420_NOR)/2; 
        }
	}
	else if (sLidarModel == D2A)
	{
		stFltLidarCfg.nBlindDist = BLND_DIST_D2;
		stFltLidarCfg.nFPS = FPS_2000_NOR;
		stFltLidarCfg.nSpinSpeed = SPEED_312_NOR;
		stFltLidarCfg.nLHSpdThres =  SPEED_312_NOR - 50;			
	}
	else if (sLidarModel == D2B)
	{
		stFltLidarCfg.nBlindDist = BLND_DIST_D2;
		stFltLidarCfg.nFPS = FPS_3000_NOR;
		stFltLidarCfg.nSpinSpeed = SPEED_360_NOR;
		stFltLidarCfg.nLHSpdThres =  SPEED_360_NOR - 50;			

	}
	else if (sLidarModel == T3A|| sLidarModel == T3B|| sLidarModel == T3C)
	{
		stFltLidarCfg.nBlindDist = BLND_DIST_T3;
		stFltLidarCfg.nFPS = FPS_TOF_NOR;
		stFltLidarCfg.nSpinSpeed = SPEED_TOF_NOR;
		stFltLidarCfg.nLHSpdThres =  SPEED_TOF_NOR - 50;			
	}
	else if (sLidarModel == D2A6|| sLidarModel == D2A8)
	{
		stFltLidarCfg.nBlindDist = BLND_DIST_D2;
		stFltLidarCfg.nFPS = FPS_2000_NOR;
		stFltLidarCfg.nSpinSpeed = SPEED_312_NOR;
		stFltLidarCfg.nLHSpdThres =  SPEED_312_NOR - 50;		
	}
	else if (sLidarModel == D2M6 || sLidarModel == D2M8)
	{
		stFltLidarCfg.nBlindDist = BLND_DIST_D2;
		stFltLidarCfg.nFPS = FPS_2000_NOR;
		stFltLidarCfg.nSpinSpeed = SPEED_312_NOR;
		stFltLidarCfg.nLHSpdThres =  SPEED_312_NOR - 50;		
	}
	else if (sLidarModel == D2B6|| sLidarModel == D2B8)
	{
		stFltLidarCfg.nBlindDist = BLND_DIST_D2;
		stFltLidarCfg.nFPS = FPS_3000_NOR;
		stFltLidarCfg.nSpinSpeed = SPEED_360_NOR;	
		stFltLidarCfg.nLHSpdThres =  SPEED_360_NOR - 50;		
	}
	else if (sLidarModel == D2S8||sLidarModel == D2S6)
	{
		stFltLidarCfg.nBlindDist = BLND_DIST_D2;
		stFltLidarCfg.nFPS = FPS_1800_NOR;
		stFltLidarCfg.nSpinSpeed = SPEED_300_NOR;
		stFltLidarCfg.nLHSpdThres =  SPEED_300_NOR - 50;		
	}
	else if (sLidarModel == D2P8|| sLidarModel == D2P6)
	{
		stFltLidarCfg.nBlindDist = BLND_DIST_D2;
		stFltLidarCfg.nFPS = FPS_2000_NOR;
		stFltLidarCfg.nSpinSpeed = SPEED_312_NOR;
		stFltLidarCfg.nLHSpdThres =  SPEED_312_NOR - 50;		
	}	
    else
    {
        return false;
    }
    return true;
}

/**************************************************************************************************************************/
// Not use
/**************************************************************************************************************************/
void HCLidarFilter::ScopeSeek(lstPtCloud_t &vcPoint,double &dAngSt, double &dAngEnd,bool bEnClrAng)
{		
	return; 		
}

/***********************************************************************************************************************************************************
// 将vcCloudAIn 和 vcCloudBIn 角度和距离都不相同的点，输出到vcCloudTsOut
/************************************************************************************************************************************************************/
void HCLidarFilter::GetDifPtCloud(const std::vector <stFltCloudD_t> &vcCloudAIn,const std::vector <stFltCloudD_t> &vcCloudBIn,std::vector <stFltCloudD_t> &vcCloudTsOut)                                       
{      
    std::vector <stFltCloudD_t> vcCloudA(vcCloudAIn);
	std::vector <stFltCloudD_t> vcCloudB(vcCloudBIn);
	
    vcCloudTsOut.clear();    
	unsigned int i,j;	
	for( i = 0; i < vcCloudA.size(); i++)    //Sel From
	{
		for(j = 0; j < vcCloudB.size(); j++)
		{			
			if(vcCloudA[i].dAngle == vcCloudB[j].dAngle && vcCloudA[i].u16Dist == vcCloudB[j].u16Dist)
			{									
				vcCloudB.erase(vcCloudB.begin()+j); //A == B, del B
				vcCloudA.erase(vcCloudA.begin()+i); //A == B, del A
				i=i-1;				
				break;				
			}
		}
		if(vcCloudB.size() ==0 ||vcCloudA.size() ==0 )
			break;		
	}
	
	for( i = 0; i < vcCloudA.size(); i++)    //Sel From
	{
		vcCloudTsOut.push_back(vcCloudA[i]);
	}

	for(j = 0; j < vcCloudB.size(); j++)
	{
		vcCloudTsOut.push_back(vcCloudB[j]);	
	}	
}

/***********************************************************************************************************************************************************
// A + B -> C 结果：vcCloudAOut
/************************************************************************************************************************************************************/
void HCLidarFilter::PlusPtCloud(const std::vector <stFltCloudD_t> &vcCloudAIn,const std::vector <stFltCloudD_t> &vcCloudBIn,std::vector <stFltCloudD_t> &vcCloudOut)                                       
{      	
	vcCloudOut.clear();
	unsigned int i,j;	
	
	for( i = 0; i < vcCloudAIn.size(); i++)    //Sel From
	{
		vcCloudOut.push_back(vcCloudAIn[i]);
	}

	for(j = 0; j < vcCloudBIn.size(); j++)
	{
		vcCloudOut.push_back(vcCloudBIn[j]);	
	}
	
}

/***********************************************************************************************************************************************************
// 从 vcCloudTs 中去除 vcToFltCloud，结果输出到vcCloudTs
/************************************************************************************************************************************************************/
void HCLidarFilter::FltoutByPtCloud(std::vector <stPtCloud_t> &vcCloudTs,const std::vector <stFltCloudD_t> &vcToFltCloud,bool bPtActiveSt)
{    
    //std::vector <stPtCloud_t> &dataTmpIn(vcCloudTs);
	unsigned int i,j;	
	for( i = 0; i < vcToFltCloud.size(); i++)    //Sel From
	{
		for(j = 0; j < vcCloudTs.size(); j++)
		{			
		    if(vcCloudTs[j].bValid != bPtActiveSt)  //invalid pt to filte 
		    { 
		        vcCloudTs.erase(vcCloudTs.begin()+j);
				j=j-1;				
				continue;
		    }
			if(vcToFltCloud[i].dAngle == vcCloudTs[j].dAngle && vcToFltCloud[i].u16Dist == vcCloudTs[j].u16Dist)
			{					
				vcCloudTs.erase(vcCloudTs.begin()+j);
				break;
			}
		}
	}	
}


/***********************************************************************************************************************************************************
// 预留代码,慎用!
// data_in     : 限定范围内，经过坐标转换，待处理点云数据流
// data_out    : 限定范围内，经过坐标转换，已处理点云数据流
// bFlt1       : false 类型1，true 类型2
// bEmptyBlindA: 盲区数据流是否清除(if data_in 包含)
// bPreAct     ：false 忽略点云预处理过程
// data_in     : 默认传入就是有效数据点
************************************************************************************************************************************************************/
void HCLidarFilter::FltN(std::vector<stFltCloudD_t> &data_in,std::vector<stFltCloudD_t> &data_out,std::vector<stFltCloudD_t> &flted_out,const stFltLidarCfg_t &stLidarPara,
                                      stFltGblSetting_t stFltGblSetting,bool bFlt1,float fDotFactor,bool bEmptyBlindA,bool bPreAct)
{
    m_mutex.lock();
	double dAngIsolution = stLidarPara.nSpinSpeed/60 * 2* CV_PI / stLidarPara.nFPS;		
	double dDistOfPtFac  = 2*sin(dAngIsolution/2); 
	
	#if EN_THRES_CALIBRTE
	#define SINGLE_DIST_OFST 8	//2.72mm
	double dCaliFac = dAngIsolution/0.010472;
	dCaliFac = dCaliFac*dCaliFac;
	#else  //Calculate
	#define  SINGLE_PT_FAC sqrt(2)*sqrt(2)		
	#endif 
	flted_out.clear();	
	data_out.assign(data_in.begin(),data_in.end());            //值给data_out
    std::vector<stFltCloudD_t> &vcPointAlias =  data_in;
	int nPointSizeOrg = 0;
    int i=0,j=0;					
	if(bPreAct) //是否预处理
    {
		std::vector<stFltCloudD_t> vcPointDataAlias(data_in);						
		nPointSizeOrg = vcPointDataAlias.size();					
		for(i = 0; i < vcPointDataAlias.size(); i++)
		{		     
			double x1 = vcPointDataAlias[i].x;
		 	double y1 = vcPointDataAlias[i].y;
		 	unsigned int u16Dist = vcPointDataAlias[i].u16Dist;
		 	double dAngle = vcPointDataAlias[i].dAngle;
		 
	     	for(j = 0; j < vcPointAlias.size(); j++)
		 	{  
				double x2 = vcPointAlias[j].x;
				double y2 = vcPointAlias[j].y;
				if(u16Dist ==  vcPointAlias[j].u16Dist  && dAngle == vcPointAlias[j].dAngle)  //是否同一点 
					continue;
		  		if(fabs(x1-x2)<(12) && fabs(y1-y2)<(12))  //此处该和转数相关联
		  		{
		  			break; // 不合适
				}
		 	}
		 	if(j<vcPointAlias.size())  //Found
		 	{				
				vcPointDataAlias.erase(vcPointDataAlias.begin()+i);
				if(vcPointDataAlias.size())
					i=i-1;
				else
					break;
		 	}
		}
        		
		if(nPointSizeOrg != vcPointDataAlias.size())
		{
			for( i = 0; i < vcPointDataAlias.size(); i++)//filter out
			{
				for(j = 0; j < data_out.size(); j++)
				{
					if(vcPointDataAlias[i].dAngle == data_out[j].dAngle && vcPointDataAlias[i].u16Dist == data_out[j].u16Dist)
					{				
					    flted_out.push_back(data_out[j]);
						data_out.erase(data_out.begin()+j);
						break;
					}
				}
			}
		}        		
	}
			
	if(bFlt1)  //for 1 
	{
		double dThresSet = 8; 
		std::vector<stFltCloudD_t> vcPoint1Alias(data_in);						
		nPointSizeOrg = vcPoint1Alias.size();					
		for( i = 0; i < vcPoint1Alias.size(); i++)
		{		     
			double x1 = vcPoint1Alias[i].x;
		 	double y1 = vcPoint1Alias[i].y;
		 	unsigned int u16Dist = vcPoint1Alias[i].u16Dist;
		 	double dAngle = vcPoint1Alias[i].dAngle;

			GetDistDotThres(vcPoint1Alias[i].u16Dist,dDistOfPtFac,dCaliFac,dThresSet);
	     	for(j = 0; j < vcPointAlias.size(); j++)
		 	{  
				double x2 = vcPointAlias[j].x;
				double y2 = vcPointAlias[j].y;
				if(u16Dist ==  vcPointAlias[j].u16Dist  && dAngle == vcPointAlias[j].dAngle)  //是否同一点 
					continue;
				if(((x1-x2)*(x1-x2)+ (y1-y2)*(y1-y2))< dThresSet)                     //此处该和转数相关联
				{		  		    
		  			break; // 不合适
				}
		 	}
		 	if(j<vcPointAlias.size())  //Found
		 	{				
				vcPoint1Alias.erase(vcPoint1Alias.begin()+i);
				if(vcPoint1Alias.size())
					i=i-1;
				else
					break;
		 	}
		}

        		
		if(nPointSizeOrg != vcPoint1Alias.size())
		{
			for( i = 0; i < vcPoint1Alias.size(); i++)//filter out
			{
				for( j = 0; j < data_out.size(); j++)
				{
					if(vcPoint1Alias[i].dAngle == data_out[j].dAngle && vcPoint1Alias[i].u16Dist == data_out[j].u16Dist)
					{					
						flted_out.push_back(data_out[j]);
						data_out.erase(data_out.begin()+j);
						break;
					}
				}
			}
		}
	
	} 
	else  //清除 2 point 
	{
		std::vector<stFltCloudD_t> vcPointGet;
		vcPointGet.clear();
		nPointSizeOrg = vcPointAlias.size();
		 
		double  dThresSet = 8;		
		for( i = 0; i < vcPointAlias.size(); i++)
	 	{		     
			double x1 = vcPointAlias[i].x;
		 	double y1 = vcPointAlias[i].y;
			bool b1Pt = true;
			GetDistDotThres(vcPointAlias[i].u16Dist,dDistOfPtFac,dCaliFac,dThresSet);			
             			
         	for(j = 0; j < vcPointAlias.size(); j++)
		 	{  
				double x2 = vcPointAlias[j].x;
				double y2 = vcPointAlias[j].y;
				if(vcPointAlias[i].u16Dist ==  vcPointAlias[j].u16Dist  && vcPointAlias[i].dAngle == vcPointAlias[j].dAngle)    //是否同一点 
					continue;

				bool b2Pt = true;
				//if(fabs(x1-x2)<SINGLE_OFST && fabs(y1-y2)<SINGLE_OFST)  // 2 pt
				if(((x1-x2)*(x1-x2)+ (y1-y2)*(y1-y2))< dThresSet)			
		  		{
		  		    b1Pt = false;
					for(int n = 0; n < vcPointAlias.size(); n++)
					{					   
						if(vcPointAlias[n].u16Dist ==	vcPointAlias[i].u16Dist  && vcPointAlias[n].dAngle == vcPointAlias[i].dAngle
							||vcPointAlias[n].u16Dist ==	vcPointAlias[j].u16Dist  && vcPointAlias[n].dAngle == vcPointAlias[j].dAngle)	  //是否同一点 
							continue;

						double x3 = vcPointAlias[n].x;
						double y3 = vcPointAlias[n].y;

				        //if(fabs(x3-x1)<SINGLE_OFST && fabs(y3-y1)<SINGLE_OFST
						//	|| fabs(x3-x2)<SINGLE_OFST && fabs(y3-y2)<SINGLE_OFST)  //发现第三点
						if(((x3-x1)*(x3-x1)+ (y3-y1)*(y3-y1))<dThresSet
						 ||((x3-x2)*(x3-x2)+ (y3-y2)*(y3-y2))<dThresSet)
						{
							b2Pt = false;
							break;
						}							
					}					
		  		}

                if(b1Pt==false && b2Pt)  //肯定2独点
                {
					vcPointGet.push_back(vcPointAlias[i]);
					vcPointGet.push_back(vcPointAlias[j]);						
					break;  //j                  
				}
				else if(b1Pt)  //单点 
				{				    
					continue;
				}
				
				else if(b1Pt==false  && b2Pt==false ) //3点 
				{
					break; //j
				}
				
		 	}

		 	if(b1Pt)  //Found 1 
		 	{						 	
				vcPointGet.push_back(vcPointAlias[i]);
		 	}	
			
	 	}

        
		for(i = 0; i < vcPointGet.size()&& nPointSizeOrg != vcPointGet.size(); i++)       //filter 
		{
			for( j = 0; j < data_out.size(); j++)
			{
				if(vcPointGet[i].dAngle == data_out[j].dAngle && vcPointGet[i].u16Dist == data_out[j].u16Dist)
				{				
				    flted_out.push_back(data_out[j]); 
					data_out.erase(data_out.begin()+j);					
					break;
				}
			}
		}
	}//end for 2-point

    if(bEmptyBlindA)
    {
		std::vector<stFltCloudD_t>::iterator it;
		for(it = data_out.begin(); it != data_out.end();)
		{  				
			if(it->u16Dist < stLidarPara.nBlindDist)
			{
			    flted_out.push_back((stFltCloudD_t)*it);
				it = data_out.erase(it);
			}
			else
				it++;			
		}		
    }
	m_mutex.unlock();
}


/***********************************************************************************************************************************************************
// 预留代码,慎用!
// data_in     : 待处理点云数据流
// data_out    : 已处理点云数据流,默认size应该0
// bFlt1       : false 类型1，true 类型2
// bEmptyBlindA: 盲区数据流是否清除(if data_in 包含)
// bPreAct     ：false 忽略点云预处理过程,注意-同一处理数据流，须保证只预处理一次，增大有效点丢失
// dAngSt      : 作用域起始角度 
// dAngEnd     : 作用域结束角度
************************************************************************************************************************************************************/
void HCLidarFilter::FltRawN(const std::vector<stPtCloud_t> data_in,std::vector<stPtCloud_t> &data_out,const stFltLidarCfg_t &stLidarPara,
                                   stFltGblSetting_t stFltGblSetting,float fSeekAngSt,float fSeekAngEnd,bool bFlt1,float fDotFactor,bool bEmptyBlindA,bool bPreAct)
{
	std::vector<stFltCloudD_t> vcPointData;
	vcPointData.clear();
	for(int i = 0; i < data_in.size(); i++)   
	{
		if(data_in[i].bValid == stFltGblSetting.bPtActiveSt && data_in[i].u16Dist <= stFltGblSetting.nActDist) //只处理 nActDist（1000mm）之内的点
		{   
		    if(data_in[i].dAngle >= fSeekAngSt && data_in[i].dAngle <= fSeekAngEnd )
		    {
			    stFltCloudD_t pTemp;
			    double angle_rad_cur = data_in[i].dAngle * CV_PI / 180;
			    unsigned short u16Dist = data_in[i].u16Dist;
			    pTemp.x = u16Dist * cos(angle_rad_cur);
			    pTemp.y = u16Dist * sin(angle_rad_cur);
			    pTemp.u16Dist = (unsigned int)u16Dist;
				pTemp.dAngle = data_in[i].dAngle;
			    vcPointData.push_back(pTemp);   
		    }
		}
	}	
	std::vector<stFltCloudD_t> vcPointDataOut;
	std::vector<stFltCloudD_t> vcFltedOut;
    FltN(vcPointData,vcPointDataOut,vcFltedOut,stLidarPara,stFltGblSetting,bFlt1,fDotFactor,bEmptyBlindA,bPreAct);
	
	data_out.assign(data_in.begin(),data_in.end());	
	FltoutByPtCloud(data_out, vcFltedOut, stFltGblSetting.bPtActiveSt);
	
}


/***********************************************************************************************************************************************************
// 预留代码,慎用!
// data_in     : 待处理点云数据流,不会被改变
// act_out     : 已处理点云数据
// bFlt1       : false 类型1，true 类型2
// bEmptyBlindA: 盲区数据流是否清除(if data_in 包含)
// bPreAct     ：false 忽略点云预处理过程
// dAngSt      : 作用域起始角度 
// dAngEnd     : 作用域结束角度
************************************************************************************************************************************************************/
void HCLidarFilter::FltRawN(const std::vector<stPtCloud_t> &data_in,std::vector<stFltCloudD_t> &act_out,std::vector<stFltCloudD_t> &flted_out,const stFltLidarCfg_t &stLidarPara,
                                   stFltGblSetting_t stFltGblSetting,float fSeekAngSt,float fSeekAngEnd,bool bFlt1,float fDotFactor,bool bEmptyBlindA,bool bPreAct)
{
	std::vector<stFltCloudD_t> vcPointData;
	vcPointData.clear();
	for(int i = 0; i < data_in.size(); i++)   //转换成xy坐标
	{
		if(data_in[i].bValid == stFltGblSetting.bPtActiveSt && data_in[i].u16Dist <= stFltGblSetting.nActDist) //只处理 nActDist（1000mm）之内的点
		{   
		    if(data_in[i].dAngle >= fSeekAngSt && data_in[i].dAngle <= fSeekAngEnd )
		    {
			    stFltCloudD_t pTemp;
			    double angle_rad_cur = data_in[i].dAngle * CV_PI / 180;
			    unsigned short u16Dist = data_in[i].u16Dist;
			    pTemp.x = u16Dist * cos(angle_rad_cur);
			    pTemp.y = u16Dist * sin(angle_rad_cur);
			    pTemp.u16Dist = u16Dist;
				pTemp.dAngle = data_in[i].dAngle;
			    vcPointData.push_back(pTemp);   //保持其不被改变
		    }
		}
	}

    FltN(vcPointData,act_out,flted_out,stLidarPara,stFltGblSetting,bFlt1,fDotFactor,bEmptyBlindA,bPreAct);
	
    	
}


/***********************************************************************************************************************************************************
// 预留代码,慎用!
// data_in     : 限定范围内，经过坐标转换，待处理点云数据流
// data_out    : 限定范围内，经过坐标转换，已处理点云数据流
// flted_out   ：被滤除掉的点
// stLidarPara : Lidar Paras
// stFltLParas : L paras
// bEmptyBlindA: 是否强清盲区数据流
// bPreAct     ：true 使能预处理, 注意同一云流，需保证只作一次处理
************************************************************************************************************************************************************/
#define bLineAreaLimit true
void HCLidarFilter::FltLn(std::vector<stFltCloudD_t> &act_in,std::vector<stFltCloudD_t> &act_out,std::vector<stFltCloudD_t> &flted_out,const stFltLidarCfg_t &stLidarPara,
                                stFltGblSetting_t stFltGblSetting,stFltLParas_t stFltLParas,bool bEmptyBlindA,bool bPreAct)
{
	m_mutex.lock();
	flted_out.clear();
	act_out.assign(act_in.begin(),act_in.end());           
    std::vector<stFltCloudD_t> &vcPointAlias =  act_in;
	int nPointSizeOrg = 0;

	int i,j,n;		
	if(bPreAct) //是否预处理
    {
		std::vector<stFltCloudD_t> vcPointDataAlias(act_in);						
		nPointSizeOrg = vcPointDataAlias.size();					
		for(i = 0; i < vcPointDataAlias.size(); i++)
		{		     
			double x1 = vcPointDataAlias[i].x;
		 	double y1 = vcPointDataAlias[i].y;
		 	unsigned int u16Dist = vcPointDataAlias[i].u16Dist;
		 	double dAngle = vcPointDataAlias[i].dAngle;
		 
	     	for(j = 0; j < vcPointAlias.size(); j++)
		 	{  
				double x2 = vcPointAlias[j].x;
				double y2 = vcPointAlias[j].y;
				if(u16Dist ==  vcPointAlias[j].u16Dist  && dAngle == vcPointAlias[j].dAngle)  //是否同一点 
					continue;
		  		if(fabs(x1-x2)<(12) && fabs(y1-y2)<(12))  //此处该和转数相关联
		  		{
		  			break; // 不合适
				}
		 	}
		 	if(j<vcPointAlias.size())  //Found
		 	{				
				vcPointDataAlias.erase(vcPointDataAlias.begin()+i);
				if(vcPointDataAlias.size())
					i=i-1;
				else
					break;
		 	}
		}

        		
		if(nPointSizeOrg != vcPointDataAlias.size())
		{
			for( i = 0; i < vcPointDataAlias.size(); i++)//filter out
			{
				for(j = 0; j < act_out.size(); j++)
				{
					if(vcPointDataAlias[i].dAngle == act_out[j].dAngle && vcPointDataAlias[i].u16Dist == act_out[j].u16Dist)
					{	
						flted_out.push_back(act_out[j]);						
						act_out.erase(act_out.begin()+j);
						break;
					}
				}
			}
		}        		
	}

	// line - remove 
	std::vector<stFltCloudD_t> vcDistPoint(act_in);
	std::vector<stFltCloudD_t> veIndex;
	std::vector<stFltCloudD_t> veIndexArr;
	std::vector<int> veRcdIdx;
	veIndexArr.clear();
	veIndex.clear();		
	veRcdIdx.clear();
			  
	for(i= 0; i < vcDistPoint.size(); i++)
	{			

	    if(vcDistPoint[i].bValidIs!=stFltGblSetting.bPtActiveSt) 
			continue;
		
		if(vcDistPoint[i].u16Dist > stFltLParas.nActDist)
			continue;
		
		double x1 = vcDistPoint[i].x;
		double y1 = vcDistPoint[i].y;			
		for(j =i+1; j <  vcDistPoint.size(); j++)
		{
			if(vcDistPoint[j].u16Dist <= stFltLParas.nActDist)// 范围限定
			{
				if(vcDistPoint[j].nfound == 0x5aa5)
					continue;

				if(vcDistPoint[j].bValidIs !=stFltGblSetting.bPtActiveSt)
					continue;
				
				//printf("overflow dAngle setting -- 1 \n");
					
				double x2 = vcDistPoint[j].x;
				double y2 = vcDistPoint[j].y;
	
				//判断位置信息
			    if(bLineAreaLimit&&((x1 && y1 && x2 < 0 && y2 < 0)	|| (x1 < 0 && y1 && x2 && y2 < 0) || (x1 < 0 && y1 < 0 && x2 && y2) || (x1 && y1 < 0 && x2 < 0 && y2)))
			    		continue;

				//unsigned int dist2 = vcDistPoint[j].u16Dist;
				//double angle2 = vcDistPoint[j].dAngle;
				
				double k = (x1 - x2)?(y1 - y2)/(x1 - x2):0;
				double b =	y1 - k*x1;
				double r =	k?(-b/k) :10000; 
			   
				//printf("b = %f r =%f \n",b,r);
				
				if(fabs(b)> stFltLParas.nCirCenFld && fabs(r) > stFltLParas.nCirCenFld)  //判断是否满足基本条件的直线	
					continue;
				
				//printf("fabs(b) = %f	fabs(r) = %f \n",fabs(b),fabs(r));
				
				int nCountFnd = 0;
				for(n = j+1; n < vcDistPoint.size(); n++)
				{
					if(vcDistPoint[n].u16Dist > stFltLParas.nActDist)
						continue;
					
					if(vcDistPoint[n].nfound == 0x5aa5)
						continue;
					
					if(vcDistPoint[n].bValidIs!=stFltGblSetting.bPtActiveSt)
						continue;
						
					double x3 = vcDistPoint[n].x;
					double y3 = vcDistPoint[n].y;
	
					//判断位置信息					
					if((bLineAreaLimit)&&((x1 && y1 && x3 < 0 && y3 < 0)|| (x1 < 0 && y1 && x3 && y3 < 0) || (x1 < 0 && y1 < 0 && x3 && y3) || (x1 && y1 < 0 && x3 < 0 && y3)))
						continue;
												
					double yc3 = x3 * k + b;							
					if(fabs(y3 - yc3) <= stFltLParas.nPtToLineMin)
					{					   
						nCountFnd++;							
						veIndex.push_back(vcDistPoint[n]);
						veRcdIdx.push_back(n);							
					}						
				}
				
				if(nCountFnd >= stFltLParas.nPtOfLineMin)	//确定找到有效值否
				{
					//veIndexArr.push_back(veIndex);
					for(int f=0;f<veIndex.size();f++)
					{
						veIndexArr.push_back(veIndex[f]);
					}
					veIndexArr.push_back(vcDistPoint[i]);
					veIndexArr.push_back(vcDistPoint[j]);
					
					for(int s=0;s<veRcdIdx.size();s++)
					{
						vcDistPoint[veRcdIdx[s]].nfound = 0x5aa5;
					}
					vcDistPoint[i].nfound = 0x5aa5;  //意义不大
					vcDistPoint[j].nfound = 0x5aa5;
					
				}
				veIndex.clear();
				veRcdIdx.clear();								
			}
		}
	}
	
	//printf("veIndexArr size: %d\n",veIndexArr.size()); 	
	if(veIndexArr.size())
	{		   
		  for(int i = 0; i < veIndexArr.size(); i++) 
		  { 		  
			  for(int j = 0; j < act_out.size(); j++)
			  {
				  if(veIndexArr[i].dAngle == act_out[j].dAngle && veIndexArr[i].u16Dist == act_out[j].u16Dist)
				  { 				  
					  //if( vcPoint[j].u16Dist <= 250)
					  {
					        flted_out.push_back(act_out[j]);							
							act_out.erase(act_out.begin()+j);							
							j=j-1;
							break;	  
					  }
				  }
			  }
		  } 	  
	}

	if(bEmptyBlindA)
    {
		std::vector<stFltCloudD_t>::iterator it;
		for(it = act_out.begin(); it != act_out.end();)
		{  				
			if(it->u16Dist < stLidarPara.nBlindDist)
			{
			    flted_out.push_back((stFltCloudD_t)*it); 
				it = act_out.erase(it);
			}
			else
				it++;			
		}		
    }	
	m_mutex.unlock();
}

/***********************************************************************************************************************************************************
// 预留代码,慎用!
// data_in     		: 待处理点云数据流
// data_out   	 	: 已处理点云数据流
// stLidarPara 		: Lidar Paras
// stFltGblSetting  : Global Setting
// stFltLParas      : L paras Setting
// bEmptyBlindA		: 是否强清盲区数据流
// bPreAct          ：true 使能预处理, 注意同一云流，需保证只作一次处理
************************************************************************************************************************************************************/
void HCLidarFilter::FltRawL(const std::vector<stPtCloud_t> data_in,std::vector<stPtCloud_t> &data_out,const stFltLidarCfg_t &stLidarPara,
                                   stFltGblSetting_t stFltGblSetting,stFltLParas_t stFltLParas,float fSeekAngSt,float fSeekAngEnd,bool bEmptyBlindA,bool bPreAct)
{
	std::vector<stFltCloudD_t> vcPointData;
	vcPointData.clear();
	for(int i = 0; i < data_in.size(); i++)   //转换成xy坐标
	{
	    stPtCloud_t sinfo =  data_in[i];  

		printf(" sinfo.dAngle = %f \n",sinfo.dAngle);
		
		if((sinfo.bValid == stFltGblSetting.bPtActiveSt) && (sinfo.u16Dist <= stFltGblSetting.nActDist) )
		{   
		    if((sinfo.dAngle >= fSeekAngSt) && (sinfo.dAngle <= fSeekAngEnd) )
		    {
			    stFltCloudD_t pTemp;
			    double angle_rad_cur = sinfo.dAngle * CV_PI / 180;
			    unsigned short u16Dist = sinfo.u16Dist;
			    pTemp.x = u16Dist * cos(angle_rad_cur);
			    pTemp.y = u16Dist * sin(angle_rad_cur);
			    pTemp.u16Dist = u16Dist;
				pTemp.dAngle = sinfo.dAngle;
			    vcPointData.push_back(pTemp);  
		    }
		}
	}	
	std::vector<stFltCloudD_t> vcPointDataOut;
	std::vector<stFltCloudD_t> vcFltedOut;
    FltLn(vcPointData,vcPointDataOut,vcFltedOut,stLidarPara,stFltGblSetting,stFltLParas,bEmptyBlindA,bPreAct);

	data_out.assign(data_in.begin(),data_in.end());	
	FltoutByPtCloud(data_out, vcFltedOut, stFltGblSetting.bPtActiveSt);

}

/***********************************************************************************************************************************************************
// 预留代码,慎用!
// data_in     		: 待处理点云数据流
// data_out   	 	: 已处理点云数据（限定区域内，过滤后的点云数据）
// stLidarPara 		: Lidar Paras
// stFltGblSetting  : Global Setting
// stFltLParas      : L paras Setting
// bEmptyBlindA		: 是否强清盲区数据流
// bPreAct          ：true 使能预处理, 注意同一云流，需保证只作一次处理
************************************************************************************************************************************************************/
void HCLidarFilter::FltRawL(const std::vector<stPtCloud_t> data_in,std::vector<stFltCloudD_t> &act_out,std::vector<stFltCloudD_t> &flted_out,const stFltLidarCfg_t &stLidarPara,
                                   stFltGblSetting_t stFltGblSetting,stFltLParas_t stFltLParas,float fSeekAngSt,float fSeekAngEnd,bool bEmptyBlindA,bool bPreAct)
{
	std::vector<stFltCloudD_t> vcPointData;
	vcPointData.clear();
	for(int i = 0; i < data_in.size(); i++)   //转换成xy坐标
	{
	    stPtCloud_t sinfo =  data_in[i];

		if((sinfo.dAngle >= fSeekAngSt) && (sinfo.dAngle <= fSeekAngEnd) )
		printf(" sinfo.dAngle = %f \n",sinfo.dAngle);
		
		if((sinfo.bValid == stFltGblSetting.bPtActiveSt) && (sinfo.u16Dist <= stFltGblSetting.nActDist) )
		{   
		    if((sinfo.dAngle >= fSeekAngSt) && (sinfo.dAngle <= fSeekAngEnd) )
		    {
			    stFltCloudD_t pTemp;
			    double angle_rad_cur = sinfo.dAngle * CV_PI / 180;
			    unsigned short u16Dist = sinfo.u16Dist;
                pTemp.x = u16Dist * cos(angle_rad_cur);
			    pTemp.y = u16Dist * sin(angle_rad_cur);
			    pTemp.u16Dist = u16Dist;
				pTemp.dAngle = sinfo.dAngle;
			    vcPointData.push_back(pTemp);  
		    }
		}
	}	

	
	FltLn(vcPointData,act_out,flted_out,stLidarPara,stFltGblSetting,stFltLParas,bEmptyBlindA,bPreAct);
	
}


/***********************************************************************************************************************************************************
// 预留代码,慎用!
// data_in     		: 待处理点云数据流
// data_out   	 	: 已处理点云数据（限定区域内，过滤后的点云数据）
// stLidarPara 		: Lidar Paras
// stFltGblSetting  : Global Setting
// stFltLParas      : L paras Setting
// bEmptyBlindA		: 是否强清盲区数据流
// bPreAct          ：true 使能预处理, 注意同一云流，需保证只作一次处理
************************************************************************************************************************************************************/

#define HALOGEN_SW_GAP 0
void HCLidarFilter::FltHalogen(const std::vector<stPtCloud_t> data_in,std::vector<stPtCloud_t> &data_out,const stFltLidarCfg_t &stLidarPara,
                                             const stFltGblSetting_t &stFltGblSetting,stFltLParas_t &stFltLParas,float fDotFactor,bool bFlt1,bool bEmptyBlindA,bool bPreAct)
{	
    #ifdef LOOSE_RUN
	static int iCount = 0;
    iCount++;
	static BOOL bFirstIn = FALSE;
    if(!bFirstIn)
	{
		if(iCount>1000)
		{
        	bFirstIn = TRUE;            
		}		 
	}	
	else
	{	 	
		if(iCount<1) //<N  return, 
		{
			data_out.clear();
	        return;				
		}
		iCount = 0;	 // >N clear, St to next loop	 
	}
	#endif
	 
    static std::vector <stPtCloud_t>  vcPointCloudLast;
    static bool bLastStrongIs = false;
	static int nStrongChkCnt = 100;	    
	
    bool bStrongIs = false;
    bool bStrongCurIs = false;
    int  nStrongCnt = 0;  
	int  dMax=0,dMin=0,nLmax=0,nLMaxInner = 0;		 
    bool bFirst = false;

    double dInnerAngSt = 0.0;
	double dInnerAngEnd = 0.0;
	bool   bByHighLight = false;
	
    double dAngleBegin = 0;
    double dAngleEnd = 0;	
	

	std::vector<stPtCloud_t> vcValidGray; 
	stPtCloud_t sInfo;
    data_out.clear();
	for(int i=0; i< data_in.size(); i++)
	{
    	sInfo = data_in.at(i);
        data_out.push_back(sInfo);
		
        if(dMax <sInfo.dAngle)
			dMax = sInfo.dAngle;
        if(dMin > sInfo.dAngle)
			dMin = sInfo.dAngle;

		if(nLmax < sInfo.u16Gray)
			nLmax = sInfo.u16Gray;
		
                if(sInfo.u16Dist < stFltGblSetting.nActDist)
		{   
		    if(sInfo.u16Gray >= stFltGblSetting.nHalogenGrayThres) 
		    {
				nStrongCnt++;
				bByHighLight = true;
		    }	
            else if(sInfo.u16Dist && sInfo.u16Dist <= (stLidarPara.nBlindDist+4) && sInfo.u16Gray >= stFltGblSetting.nSharkGrayThres)
			{
				if(dInnerAngSt==0)
					dInnerAngSt = sInfo.dAngle;
				if(dInnerAngEnd==0)
					dInnerAngEnd = sInfo.dAngle;					
				nStrongCnt++;				
				if(nLMaxInner < sInfo.u16Gray)
					nLMaxInner = sInfo.u16Gray;
				
				vcValidGray.push_back(sInfo);
			}			
        }
    }

	if(nStrongCnt>=1)
	{	     	    
    	bStrongIs = true;		
	}
	
	int nThres = HALOGEN_SW_GAP;
	bool bSteadyIs = true;
	if(bStrongIs != bLastStrongIs)  
	{
	      
		if(nThres>3) 
			nThres =3;
		
		if(nStrongChkCnt == 100)
			nThres = 0;
		nStrongChkCnt++;
		if(nStrongChkCnt > nThres)
		{
			bLastStrongIs = bStrongIs;		
		}
		else
		{				
			int c;
			for(c=0;c<data_out.size();c++)
			{				
				sInfo = data_out[c];
				if(sInfo.u16Dist <=stFltGblSetting.nActDist) //clear 1000 domain
				{
					data_out.erase(data_out.begin()+c);					
					c=c-1;
				}				
			}
					    
			for(c=0;c < vcPointCloudLast.size();c++)
			{
			    sInfo = vcPointCloudLast[c];
			    if(sInfo.u16Dist <=stFltGblSetting.nActDist)
			    {				
					data_out.push_back(sInfo);
			    }
			}
		  	bSteadyIs = false; 			
		}
	}

    //keep last F
    if(bSteadyIs)
    {
        if(HALOGEN_SW_GAP)
        {
		    vcPointCloudLast.clear(); 
			for(int c=0;c < data_out.size();c++)
			{
			    sInfo = data_out[c];
				if(sInfo.u16Dist <= stFltGblSetting.nActDist)
					vcPointCloudLast.push_back(sInfo);			
			}		
        }
		nStrongChkCnt = 0;
		bStrongCurIs = bStrongIs;
    }
	
		
    #if 0          
		if(bStrongCurIs)
		{
		    if(bByHighLight==false)
				printf("-------------Inner High -------nLmax = %d ------\n",nLMaxInner);
			else
				printf("---------------High -------nLmax = %d ------\n",nLmax);
		}
		else
			printf("---------------Normal  ------------nLmax = %d ------\n",nLmax);
    #endif
	
        double dCenter;
		if(bStrongCurIs && bByHighLight==false)
		{            			
			if(vcValidGray.size())
			{
				sort(vcValidGray.begin(), vcValidGray.end(), comTSAng);   //排序 
				dInnerAngSt = vcValidGray[0].dAngle;
				dInnerAngEnd = vcValidGray[vcValidGray.size()-1].dAngle;  
                                dCenter = (dInnerAngSt+dInnerAngEnd)/2.0;
				dAngleBegin = dCenter - stFltGblSetting.nAngExtent;
				dAngleEnd = dCenter + stFltGblSetting.nAngExtent;
			}
		
            dCenter = (dInnerAngSt+dInnerAngEnd)/2.0; 
			if(fabs(dInnerAngSt-dInnerAngEnd) > 2*stFltGblSetting.nAngExtent)
	 	    {
				dAngleBegin = dCenter - fabs(dInnerAngSt-dInnerAngEnd)-5;
		  		dAngleEnd = dCenter +  fabs(dInnerAngSt-dInnerAngEnd) + 5;
	 	    }
		    //else if (dInnerAngSt == dInnerAngEnd) 
			//{
			//	dAngleBegin = dCenter - ANGLE_OFST_MAX;
			//	dAngleEnd = dCenter + ANGLE_OFST_MAX;				
			//}
		}
		else
		{				
            if(data_out.size())
            {
				std::vector<stPtCloud_t> tmp0(data_out);  
			  	sort(tmp0.begin(), tmp0.end(), comTSGray);        //排序 亮度
			    
			  	dCenter = tmp0[0].dAngle;                 	    	
				dAngleBegin = dCenter - stFltGblSetting.nAngExtent;
				dAngleEnd = dCenter + stFltGblSetting.nAngExtent;
            }
		}
		
		if(dAngleEnd > 360)
		{
			dAngleEnd = dAngleEnd - 360;
		}
	  	if(dAngleBegin < 0)
		{
	  		dAngleBegin = 360 + dAngleBegin;
	  	}
           
	  	if(bStrongCurIs==false) 
	  	{
      		dAngleBegin = 0;
            dAngleEnd = 0;
	  	}          
		
		//tmp became the global data to deal  
		std::vector<stPtCloud_t> tmp(data_out);	
		std::vector<stPtCloud_t>::iterator it;		
        if(0)
		for(it = tmp.begin(); it != tmp.end();)
        {
            if((dAngleBegin <= dAngleEnd) && (it->dAngle >= dAngleBegin && it->dAngle <= dAngleEnd)
				||(dAngleBegin > dAngleEnd) && (it->dAngle >= dAngleBegin || it->dAngle <= dAngleEnd)
				|| it->u16Dist <= stLidarPara.nBlindDist  //seems need，why 
				)
			{
              	it = tmp.erase(it);				  
            }
            else
			{				
            	it++;
            }
        }
		
		//flt Ln
	    std::vector<stFltCloudD_t> vcPointDataOutL;
		std::vector<stFltCloudD_t> vcFltOutL;
		FltRawL(tmp, vcPointDataOutL,vcFltOutL, stLidarPara,stFltGblSetting,stFltLParas,dAngleBegin,dAngleEnd,false,false);
		
 
     	//flt N	
		std::vector<stFltCloudD_t> vcPointDataOutN;
		std::vector<stFltCloudD_t> vcFltOutN;
		FltN(vcPointDataOutL,vcPointDataOutN,vcFltOutN,stLidarPara, stFltGblSetting,fDotFactor,bFlt1,false,bPreAct);

		data_out.clear();			
		data_out.assign(tmp.begin(),tmp.end()); 

        std::vector<stFltCloudD_t> vcFltOut;
		PlusPtCloud(vcFltOutL, vcFltOutN, vcFltOut);	
		FltoutByPtCloud(data_out, vcFltOut, stFltGblSetting.bPtActiveSt);
		
		
		int  nInnerIDot = 0;
		int  nInnerODot = 0;
		for (it = data_out.begin(); it != data_out.end();)
		{
			if (it->u16Dist >= stLidarPara.nBlindDist && it->u16Dist < (stFltGblSetting.nActDist - 200) && it->bValid==stFltGblSetting.bPtActiveSt)
				nInnerIDot++;
			if (it->u16Dist >= stLidarPara.nBlindDist && it->u16Dist < (stFltGblSetting.nActDist - 100) && it->bValid==stFltGblSetting.bPtActiveSt)
				nInnerODot++;
			if (it->u16Dist < stLidarPara.nBlindDist)
			{
				it = data_out.erase(it);
			}
			else
				it++;
		}
		
		if (nInnerIDot < 4 && nInnerIDot == nInnerODot && nInnerODot)
		{
			for (it = data_out.begin(); it != data_out.end();)
			{
				if (it->u16Dist >= stLidarPara.nBlindDist && it->u16Dist < (stFltGblSetting.nActDist - 100))
					it = data_out.erase(it);
				else
					it++;
			}
		}
}




