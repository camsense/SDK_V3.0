#pragma once
#include "pointCloudFilters.h"
#include <vector>
#include <assert.h>
#include <unordered_map>

#define  FLT_VER                    (char*)"1.0.1"   

#define  N_ANG 0                      //1 N-degree；0 One-circle
#define  EN_THRES_CALIBRTE            //function option
#define  EN_INHERIT           1       //function option
#define  EN_ANG_LIM           1       //function with angle limit

#define  CV_PI                3.1415926535897932384626433832795


/**************************************************************************************************************************/
//quick sort in dist 
/**************************************************************************************************************************/
bool comP2DDist(Point2D&a, Point2D&b)
{
	if (a.dist > b.dist)
	{
		return true;
	}
	else
	{
		return false;
	}
}
/**************************************************************************************************************************/
//quick sort in angle,H->L 
/**************************************************************************************************************************/
bool comP2DAng(Point2D& s1, Point2D& s2)
{
	return s1.angle > s2.angle;
}


/**************************************************************************************************************************/
//quick sort in grey,H->L 
/**************************************************************************************************************************/
bool comTSGray(tsPointCloud&a, tsPointCloud&b)
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
//quick sort in angle H->L 
/**************************************************************************************************************************/
bool comTSAng(const tsPointCloud& s1, const tsPointCloud& s2)

{
	return s1.dAngle > s2.dAngle;
}


//*****************************************************************************
//Demo of function calling
//Input ：
//       lstPointCloud - whole circle clouds data
//       giFilterSpeed - motor speed, RPM,  eg. X2B/318, X2M/318, D2S/300, X2MF/360 ,etc.     
//       giFilterFPS - FPS,  eg. X2B/2200, X2M/2200,D2S/1800, X2MF/2080 ,etc.   
//*****************************************************************************
LstPointCloud pointCloudFilters::hotPixelFilter(LstPointCloud& lstPointCloud,int giFilterSpeed, int giFilterFPS)
{	
	m_mutex.lock();	
	//sort(lstPointCloud.begin(), lstPointCloud.end(), comTSAng);  	 
	hotpixelFilterDo(lstPointCloud,giFilterSpeed,giFilterFPS);	
	m_mutex.unlock();	
	return m_pointCloud;
}

#define  GRAY_DIST_LIMIT    800      //Distance to get Halogen light, if need to adj.;
#define  GRAY_RIGHT_MAX     7000     //Feature of Halogen, if need to adj.;   
#define  HALOGEN_DETECT_ADJ 0        //mm dist ext. to find halogen feature,if with Blind-Area type. if need to adj.

#define  ANG_COMP_OFST      50       //Angle offset value +/-
#define  H_L_SPD_THRES      270      //Threshold of High/Low Spin-speed,design value;Eg. X2MF(180/360 L/H RMS) : 270,  X2F(360 RMS):270,D2S(300):210    
#define  DIST_OUTER_CIRCLE  1000     //mm Range-limit,may Adjust as required
#define  DIST_INNER_CIRCLE  120      //mm Blind area size; eg. D2?:140, X2? 120, X1?100 
#define  FAC_HSPD_ADJ       3.0      //for high RPM,it is to spots which are filtered out  or left; need precise  adj. more
#define  FAC_LSPD_ADJ       3.0      //for low RPM,it is to spots which are filtered out  or left; need precise  adj. more

/**************************************************************************************************************************/
// Interface of Strong-light
// Input: 
//       lstPointCloud - Whole circle DotClouds data
//       giFilterSpeed - RPM
//       giFilterFPS - FPS 
// OutPut:
//   m_pointCloud - Result of DotCloud list
/**************************************************************************************************************************/
LstPointCloud pointCloudFilters::strongLightFilterDo(LstPointCloud& lstPointCloud,int giFilterSpeed, int giFilterFPS)
{
	bool    bStrongIs = false;
	bool    bStrongCurIs = false;
	int     nStrongCnt = 0;
	int     dMax = 0, dMin = 0, nLmax = 0, nLMaxInner = 0;
	int     nBegin = 0, nEnd = 0;
	bool    bFirst = false;

	double  dInnerAngSt = 0.0;
	double  dInnerAngEnd = 0.0;
	bool    bByHighLight = false;

	double  dAngleBegin = 0;
	double  dAngleEnd = 0;
	tsPointCloud sInfo;

	int biS[4]={0,0,0,0};
	m_pointCloud.clear();
	for (int i = 0; i < lstPointCloud.size(); i++)
	{
		sInfo = lstPointCloud[i];
		m_pointCloud.push_back(sInfo);

		if (dMax < sInfo.dAngle)
			dMax = sInfo.dAngle;
		if (dMin > sInfo.dAngle)
			dMin = sInfo.dAngle;

		if (nLmax < sInfo.u16Gray)
			nLmax = sInfo.u16Gray;

		if (sInfo.u16Dist < GRAY_DIST_LIMIT)
		{
			if (sInfo.u16Gray >= GRAY_RIGHT_MAX)
			{
				nStrongCnt++;
				bByHighLight = true;
			}
			else  if (sInfo.u16Dist && sInfo.u16Dist <= (DIST_INNER_CIRCLE + HALOGEN_DETECT_ADJ)  && sInfo.u16Gray >= 50)
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

	if(nStrongCnt >= 1)
	{
		bStrongIs = true;
	}
 
	double dCenter = 0.0;		
	if (bStrongIs && bByHighLight == false)  //盲区杂点方式
	{            
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
					//dInnerAngSt =  
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
	   	}
		else
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
		}
	}
	else 
	{
		LstPointCloud tmp0(m_pointCloud);
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

	LstPointCloud tmp(m_pointCloud);
	if(DIST_INNER_CIRCLE)
	{
		LstPointCloud::iterator it;
		for (it = tmp.begin(); it != tmp.end();)
		{  

			if (it->u16Dist <= DIST_INNER_CIRCLE)
			{
				it = tmp.erase(it);
			}
			else
				it++;
		} 
    }
	checkPoint(tmp, 2, dAngleBegin, dAngleEnd,giFilterSpeed,giFilterFPS,FAC_HSPD_ADJ,FAC_LSPD_ADJ);	
	return m_pointCloud;
}

/**************************************************************************************************************************/
// Interface of normal scene
// Input: 
//       lstPointCloud - Whole circle DotClouds data
//       giFilterSpeed - RPM 
//       giFilterFPS - FPS 
// OutPut:
//       m_pointCloud - Result of DotCloud list
/**************************************************************************************************************************/
LstPointCloud pointCloudFilters::hotpixelFilterDo(LstPointCloud& lstPointCloud,int giFilterSpeed, int giFilterFPS)
{
	m_pointCloud.clear();
	for (int i = 0; i < lstPointCloud.size(); i++)
	{
		m_pointCloud.push_back(lstPointCloud[i]);
	}
	LstPointCloud tmp(m_pointCloud);
	if(DIST_INNER_CIRCLE)
	{
		LstPointCloud::iterator it;
		for (it = tmp.begin(); it != tmp.end();)
		{  

			if (it->u16Dist <= DIST_INNER_CIRCLE)
			{
				it = tmp.erase(it);
			}
			else
				it++;
		} 
    }	
	hotPixelCheck(lstPointCloud, 2, 0, 0,giFilterSpeed,giFilterFPS,FAC_HSPD_ADJ,FAC_LSPD_ADJ);
	return m_pointCloud;
}



#define  ORG_REGION          65.0     //mm, Ray valid area,Better to opt.till now need not; 
#define  DIST_LN_CLEAR_MAX   600      //mm, Line Clear Max,Better to opt       
#define  LINAR_OFST_MAX      3        //Max offset of Spot to the ray 
#define  LINAR_CNT           3        //Min Spots of Ray 
#define  CONFIDENCE_THR      7        //confidence level of High light disappearing

//**************************************************************************************************************************/
//Function of normal scene
// Input: 
//       lstPointCloud - Whole circle DotClouds data
//       nWkType - No use
//       dAngSt - Halogen Angle
//       dAngEnd - Halogen Angle 
//       giFilterSpeed - RPM
//       giFilterFPS - FPS
//       fFacAdjHSpd - Factor of Dot distance Adj. for high RPM
//       fFacAdjLSpd - Factor of Dot distance Adj. for low RPM
// OutPut:
//       m_pointCloud - DotCloud list
//**************************************************************************************************************************/
void pointCloudFilters::hotPixelCheck(LstPointCloud lstPointCloud, int nWkType, double dAngSt, double dAngEnd, int giFilterSpeed,int giFilterFPS,float fFacAdjHSpd,float fFacAdjLSpd)
{
 	float fFacAdj = fFacAdjHSpd;		
	LstPointCloud vcPoint(lstPointCloud);
	vector<Point2D> vcPointData;
	vcPointData.clear();
	for (int i = 0; i < lstPointCloud.size(); i++)
	{
		if (lstPointCloud[i].u16Dist && lstPointCloud[i].u16Dist <= DIST_OUTER_CIRCLE)
		{   					
			Point2D pTemp;
			double angle_rad_cur = lstPointCloud[i].dAngle * CV_PI / 180;
			double dist = lstPointCloud[i].u16Dist;
			pTemp.x = dist * cos(angle_rad_cur);
			pTemp.y = dist * sin(angle_rad_cur);
			pTemp.dist = dist;
			pTemp.angle = lstPointCloud[i].dAngle;
			pTemp.nfound = 0;
			vcPointData.push_back(pTemp);			
		}					
	}    		  
	sort(vcPointData.begin(), vcPointData.end(), comP2DAng);   
	FltL(vcPoint,vcPointData,dAngSt, dAngEnd);
	
  #if EN_INHERIT==0  
	Flt1Pt(vcPoint,vcPointData,dAngSt, dAngEnd, giFilterSpeed,giFilterFPS,fFacAdj); 			   	
	Flt2Pt(vcPoint,vcPointData,dAngSt, dAngEnd, giFilterSpeed,giFilterFPS,fFacAdj,EN_ANG_LIM? true:false); 	   	
  #elif EN_INHERIT==1
	Flt1Pt(vcPoint,vcPointData,dAngSt, dAngEnd, giFilterSpeed,giFilterFPS,fFacAdj); 		   	
  #endif
	m_pointCloud.swap(vcPoint);	
}

/**************************************************************************************************************************/
// Fucntion of strong-light 
// Input: 
//       lstPointCloud - Whole circle DotClouds data
//       nWkType - No use
//       dAngSt - Halogen Angle
//       dAngEnd - Halogen Angle 
//       giFilterSpeed - RPM
//       giFilterFPS - FPS
//       fFacAdjHSpd - Dot distance Adj. of high RPM
//       fFacAdjLSpd - Dot distance Adj. of low RPM
// OutPut:
//       m_pointCloud - DotCloud list
//**************************************************************************************************************************/

static int nLastSpd = 0;
static int nFltRobustChk = 0;
static double dAngStLast=0x1e1e;
static double dAngEndLast=0x1e1e; 
void pointCloudFilters::checkPoint(LstPointCloud lstPointCloud, int nWkType, double dAngSt, double dAngEnd, int giFilterSpeed,int giFilterFPS,float fFacAdjHSpd,float fFacAdjLSpd)
{
    bool bHSpd = false;
	float fFacAdj = fFacAdjHSpd;	
    if(nLastSpd==0)   
    	nLastSpd = giFilterSpeed;
    else
    {      
		if(nLastSpd > H_L_SPD_THRES)  
		{
			bHSpd = true;
			fFacAdj = fFacAdjHSpd;       
		}
		else 
		{
			bHSpd = false;
			fFacAdj = fFacAdjLSpd;
		}
		
		if(nLastSpd != giFilterSpeed)  
		{
			dAngStLast = 0x1e1e;
			dAngEndLast = 0x1e1e;
		}			 
		nLastSpd = giFilterSpeed;
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

		if(dAngStEst <= 180 && fabs(360-dAngEndEst + dAngStEst)<181)  
        {						
			dAngStEst = dAngStEst+ANG_COMP_OFST;
			dAngEndEst = dAngEndEst-ANG_COMP_OFST;
									
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
			dAngStEst = dAngStEst-ANG_COMP_OFST;
			dAngEndEst = dAngEndEst+ANG_COMP_OFST;
			
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
	LstPointCloud vcPoint(lstPointCloud);
	vector<Point2D> vcPointData;
	vcPointData.clear();
	
	for (int i = 0; i < lstPointCloud.size(); i++)
	{
		if (lstPointCloud[i].u16Dist && lstPointCloud[i].u16Dist <= DIST_OUTER_CIRCLE)
		{   					
			Point2D pTemp;
			double angle_rad_cur = lstPointCloud[i].dAngle * CV_PI / 180;
			double dist = lstPointCloud[i].u16Dist;
			pTemp.x = dist * cos(angle_rad_cur);
			pTemp.y = dist * sin(angle_rad_cur);
			pTemp.dist = dist;
			pTemp.angle = lstPointCloud[i].dAngle;
			pTemp.nfound = 0;
			vcPointData.push_back(pTemp);			
		}
					
	}    		  
	sort(vcPointData.begin(), vcPointData.end(), comP2DAng);   

	if(nFltRobustChk)
		FltL(vcPoint,vcPointData,dAngStLast, dAngEndLast);
	
  #if EN_INHERIT==0  
	Flt1Pt(vcPoint,vcPointData,dAngStLast, dAngEndLast, giFilterSpeed,giFilterFPS,fFacAdj); 			   	
	if(nFltRobustChk)
		Flt2Pt(vcPoint,vcPointData,dAngStLast, dAngEndLast, giFilterSpeed,giFilterFPS,fFacAdj,EN_ANG_LIM? true:false); 	   	
  #elif EN_INHERIT==1
	Flt1Pt(vcPoint,vcPointData,dAngStLast, dAngEndLast, giFilterSpeed,giFilterFPS,fFacAdj); 	
	//if(nFltRobustChk)
		FltNPt(vcPoint,vcPointData,dAngStLast, dAngEndLast, giFilterSpeed,giFilterFPS,fFacAdj,3,EN_ANG_LIM? true:false);  
	if(nFltRobustChk)
		Flt2Pt(vcPoint,vcPointData,dAngStLast, dAngEndLast, giFilterSpeed,giFilterFPS,fFacAdj,EN_ANG_LIM? true:false); 	   	
  #endif
	m_pointCloud.swap(vcPoint);
	
}


/**************************************************************************************************************************/
// Ray line to filter out 
// Input: 
//       vcPoint - Whole circle DotClouds data
//       vcPointData - 1M X/Y DotClouds
//       dAngSt  - No use,reserved
//       dAngEnd - No use,reserved
// OutPut:
//       vcPoint - Whole circle DotClouds data
//       vcPointData - Changed for next step
/**************************************************************************************************************************/
void pointCloudFilters::FltL(LstPointCloud &vcPoint,vector<Point2D> &vcPointData,double &dAngSt, double &dAngEnd)
{    
	vector<Point2D> &vcDistPoint = vcPointData;
	vector<Point2D> veIndex;
	unordered_map<string, Point2D> veIndexArr;
	veIndexArr.clear();
	veIndex.clear();
	vector<int> veRcdIdx;
	veRcdIdx.clear();
	bool bNormal = true;
	double dASt = 0, dAEnd = 0;
	int i, j, n;
	{

		for (i = 0; i < vcDistPoint.size(); i++)
		{
			if (vcDistPoint[i].dist > DIST_LN_CLEAR_MAX)
				continue;

			if (vcDistPoint[i].nfound == 0x5aa5)
				continue;

			dASt = vcDistPoint[i].angle - 15;
			dAEnd = vcDistPoint[i].angle + 15;
			if (dAEnd >= 360)
			{
				dAEnd = dAEnd - 360;
			}
			if (dASt < 0)
			{
				dASt = 360 + dASt;
			}

			double x1 = vcDistPoint[i].x;
			double y1 = vcDistPoint[i].y;

			for (j = 0; j < vcDistPoint.size(); j++)
			{
				if (j == i)
					continue;

				if (vcDistPoint[j].dist <= DIST_LN_CLEAR_MAX)
				{
					//if (vcDistPoint[j].nfound == 0x5aa5) 
					//	continue;

					if (angleInRange(vcDistPoint[j].angle, dASt, dAEnd) == false)
						continue;

					double x2 = vcDistPoint[j].x;
					double y2 = vcDistPoint[j].y;
					double k = (x1 - x2) ? (y1 - y2) / (x1 - x2) : 0;

					double b = y1 - k * x1;
					//double r = k ? (-b / k) : 10000;

					double dtoorg = (b) / sqrt(k*k + 1);
					if (fabs(dtoorg) > ORG_REGION)
						continue;

					int nCountFnd = 0;
					for (n = 0; n < vcDistPoint.size(); n++)
					{
						if (vcDistPoint[n].dist > DIST_LN_CLEAR_MAX)
							continue;

						if (n == i || n == j)
							continue;

						//if (vcDistPoint[n].nfound == 0x5aa5)
						//	continue;

						if (angleInRange(vcDistPoint[n].angle, dASt, dAEnd) == false)
							continue;

						double x3 = vcDistPoint[n].x;
						double y3 = vcDistPoint[n].y;
						double yc3 = (k*x3 - y3 + b) / sqrt(k*k + 1);
						if (fabs(yc3) <= LINAR_OFST_MAX)
						{
							nCountFnd++;
							veIndex.push_back(vcDistPoint[n]);
							veRcdIdx.push_back(n);
						}
					}

					if (nCountFnd >= LINAR_CNT)
					{
						int f;
						int nok = 1;
						sort(veIndex.begin(), veIndex.end(), comP2DDist);
						int dst0 = veIndex[0].dist;
						double angst0 = veIndex[0].angle;
						for (f = 1; f < veIndex.size(); f++)
						{
							if (abs(int(veIndex[f].dist - dst0)) >= LINAR_OFST_MAX) //|| fabs(veIndex[f].angle - angst0)>=2.4)
							{
								nok++;
								dst0 = veIndex[f].dist;
								angst0 = veIndex[f].angle;
							}
							if (nok >= LINAR_CNT)
								break;
						}
						if (nok >= LINAR_CNT)
						{
							for (f = 0; f < veIndex.size(); f++)
							{
								veIndexArr.insert({ to_string(veIndex[f].x) + to_string(veIndex[f].y), veIndex[f] });
							}
							veIndexArr.insert({ to_string(vcPointData[i].x) + to_string(vcPointData[i].y), vcPointData[i] });
							veIndexArr.insert({ to_string(vcPointData[j].x) + to_string(vcPointData[j].y), vcPointData[j] });

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
	}

	{
		if (veIndexArr.size())
		{
			auto findItem = [=](const tsPointCloud& it) {
				for (const auto& veIndexIt : veIndexArr)
				{
					if (veIndexIt.second.angle == it.dAngle && veIndexIt.second.dist == it.u16Dist) {
						return true;
					}
				}
				return false;
			};
			for (auto it = vcPoint.begin(); it != vcPoint.end();) {
				if (findItem(*it)) {
					it = vcPoint.erase(it);
				}
				else {
					++it;
				}
			}
		}

		for (int k = 0; k < vcDistPoint.size(); k++)
		{
			if (vcDistPoint[k].nfound == 0x5aa5)
			{
				vcDistPoint.erase(vcDistPoint.begin() + k);
				k = k - 1;
			}
		}
	}
			
}


/**************************************************************************************************************************/
// One dot to filter out  
// Input: 
//       vcPoint - Whole circle DotClouds data
//       vcPointData - 1M X/Y DotClouds
//       dAngSt  - No use,reserved
//       dAngEnd - No use,reserved
//       giFilterSpeed - RPM
//       giFilterFPS - FPS
//       fFacAdj - Dot distance Adj. 
// OutPut:
//       vcPoint - Whole circle DotClouds data
//       vcPointData - Changed for next step
/**************************************************************************************************************************/
void pointCloudFilters::Flt1Pt(LstPointCloud &vcPoint,vector<Point2D> &vcPointData,double &dAngSt, double &dAngEnd, int giFilterSpeed,int giFilterFPS,float fFacAdj)
{
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

    int nThresSet = 8; 
	std::vector<Point2D> &vcPoint1Alias = vcPointData;						
    int j=0,i=0;
	
	for(i = 0; i < vcPoint1Alias.size(); i++)
	{			  
		double x1 = vcPoint1Alias[i].x;
		double y1 = vcPoint1Alias[i].y;
		
		unsigned int dist = vcPoint1Alias[i].dist;
		double angle = vcPoint1Alias[i].angle;
		 
		GetDistDotThres(vcPoint1Alias[i].dist,dDistOfPtFac,dCaliFac,nThresSet);
       
		for(j = i+1; j < vcPoint1Alias.size(); j++)
		{						   
			double x2 = vcPoint1Alias[j].x;
			double y2 = vcPoint1Alias[j].y;
			//if(dist ==  vcPoint1Alias[j].dist  && angle == vcPoint1Alias[j].angle)  //same pt or not 
			//	continue;
			
			if(((x1-x2)*(x1-x2)+ (y1-y2)*(y1-y2))< nThresSet)//relate to spin speed
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
			if(vcPoint1Alias[i].angle == vcPoint[j].dAngle && vcPoint1Alias[i].dist == vcPoint[j].u16Dist)
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
// Two dots to filter out  
// Input: 
//       vcPoint - Whole circle DotClouds data
//       vcPointData - 1M X/Y DotClouds
//       dAngSt  - Start-Angle of action
//       dAngEnd - End-Angle of action
//       giFilterSpeed - RPM
//       giFilterFPS - FPS
//       fFacAdj - Dot distance Adj. 
//       bEnAngLmt - true to en. angle limitation
// OutPut:
//       vcPoint - Whole circle DotClouds data
//       vcPointData - Changed for next step
/**************************************************************************************************************************/

void pointCloudFilters::Flt2Pt(LstPointCloud &vcPoint,vector<Point2D> &vcPointData,double &dAngSt, double &dAngEnd, int giFilterSpeed,int giFilterFPS,
                             float fFacAdj,bool bEnAngLmt)
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

	vector<Point2D> vcPointAlias(vcPointData);
	int j = 0;
	 bool bNormal = true ;
	 if(dAngSt <= 180 && fabs(360-dAngEnd + dAngSt)<181)		
		bNormal = false;

	vector<Point2D> vcPointGet;
	vcPointGet.clear();

	int  nThresSet = 8;
	for (int i = 0; i < vcPointAlias.size(); i++)
	{
	//#if EN_ANG_LIM
		if(bEnAngLmt)
		{
			if(bNormal)	
			{
		     	if (vcPointAlias[i].angle < dAngSt || vcPointAlias[i].angle > dAngEnd)
					continue;						
			}
			else
			{
				if (vcPointAlias[i].angle > dAngSt && vcPointAlias[i].angle < dAngEnd)
					continue;							
			}
		}
	//#endif	
										     
		double x1 = vcPointAlias[i].x;
		double y1 = vcPointAlias[i].y;

		GetDistDotThres(vcPointAlias[i].dist,dDistOfPtFac,dCaliFac,nThresSet);
				
		bool b1Pt = true;
		for (j = 0; j < vcPointAlias.size(); j++)
		{
            //#if EN_ANG_LIM
            if(bEnAngLmt)
            {
				if(bNormal)	
				{
		         	if (vcPointAlias[j].angle < dAngSt || vcPointAlias[j].angle > dAngEnd)
						continue;						
				}
				else
				{
					if (vcPointAlias[j].angle > dAngSt && vcPointAlias[j].angle < dAngEnd)
						continue;							
				}
            }
			//#endif
			
			double x2 = vcPointAlias[j].x;
			double y2 = vcPointAlias[j].y;
			if (vcPointAlias[i].dist == vcPointAlias[j].dist  && vcPointAlias[i].angle == vcPointAlias[j].angle)	  // 
				continue;

			bool b2Pt = true;
			if (((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2)) < nThresSet)
			{
				b1Pt = false;
				for (int n = 0; n < vcPointAlias.size(); n++)
				{
					if (vcPointAlias[n].dist == vcPointAlias[i].dist  && vcPointAlias[n].angle == vcPointAlias[i].angle
						|| vcPointAlias[n].dist == vcPointAlias[j].dist  && vcPointAlias[n].angle == vcPointAlias[j].angle)	  // 
						continue;
                    //#if EN_ANG_LIM
                    if(bEnAngLmt)
                    {
						if(bNormal)	
						{
		                 	if (vcPointAlias[n].angle < dAngSt || vcPointAlias[n].angle > dAngEnd)
								continue;						
						}
						else
						{
							if (vcPointAlias[n].angle > dAngSt && vcPointAlias[n].angle < dAngEnd)
								continue;							
						}
                    }
					//#endif

					double x3 = vcPointAlias[n].x;
					double y3 = vcPointAlias[n].y;

					if (((x3 - x1)*(x3 - x1) + (y3 - y1)*(y3 - y1)) < nThresSet
						|| ((x3 - x2)*(x3 - x2) + (y3 - y2)*(y3 - y2)) < nThresSet)
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
			if (vcPointGet[i].angle == vcPoint[j].dAngle && vcPointGet[i].dist == vcPoint[j].u16Dist)
			{
				vcPoint.erase(vcPoint.begin()+j);
				break;
			}
		}
#if EN_INHERIT==1
		for(int k = 0; k < vcPointData.size(); k++)
		{
			if(vcPointGet[i].angle == vcPointData[k].angle && vcPointGet[i].dist == vcPointData[k].dist)
			{					 
				vcPointData.erase(vcPointData.begin()+k);
				break;
			}			
		}	
#endif
		
	}
}


/**************************************************************************************************************************/
// 3 more Dots to filter out
// Input: 
//       vcPoint - Whole circle DotClouds data
//       vcPointData - 1M X/Y DotClouds
//       dAngSt  - Start-Angle of action
//       dAngEnd - End-Angle of action
//       giFilterSpeed - RPM
//       giFilterFPS - FPS
//       fFacAdj - Dot distance Adj.
//       nTrace - Dot number to trace
//       bEnAngLmt - true to en. angle limitation 
// OutPut:
//       vcPoint - Whole circle DotClouds data
//       vcPointData - Changed for next step
/**************************************************************************************************************************/
void pointCloudFilters::FltNPt(LstPointCloud &vcPoint,vector<Point2D> &vcPointData,double &dAngSt, double &dAngEnd, int giFilterSpeed,
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

	vector<Point2D> vcPointAlias(vcPointData);
	int j = 0,i=0,k=0;
	 bool bNormal = true ;
	 if(dAngSt <= 180 && fabs(360-dAngEnd + dAngSt)<181)		
		bNormal = false;

	vector<Point2D> vcPointGet;
	vcPointGet.clear();
	int  nThresSet = 8;


    bool bSeek = false;
    int is=0,ie =0;	
	for ( i = 0; i < vcPointAlias.size(); i++)
	{
	//#if EN_ANG_LIM
		if(bEnAngLmt)
		{
	
			if(bNormal)	
			{
		     	if (vcPointAlias[i].angle < dAngSt || vcPointAlias[i].angle > dAngEnd)
					continue;						
			}
			else
			{
				if (vcPointAlias[i].angle > dAngSt && vcPointAlias[i].angle < dAngEnd)
					continue;							
			}
		}	
	//#endif	
										     
		double x1 = vcPointAlias[i].x;
		double y1 = vcPointAlias[i].y;
		
		double x01 = x1;
		double y01 = y1;

		GetDistDotThres(vcPointAlias[i].dist,dDistOfPtFac,dCaliFac,nThresSet);

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
		         	if (vcPointAlias[j].angle < dAngSt || vcPointAlias[j].angle > dAngEnd)
						continue;						
				}
				else
				{
					if (vcPointAlias[j].angle > dAngSt && vcPointAlias[j].angle < dAngEnd)
						continue;							
				}
	        }
	//#endif			
			double x2 = vcPointAlias[j].x;
			double y2 = vcPointAlias[j].y;
			
			if (vcPointAlias[i].dist == vcPointAlias[j].dist  && vcPointAlias[i].angle == vcPointAlias[j].angle)  //seems go for nothing	   
				continue;				

			if (((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2)) < nThresSet  || ((x01 - x2)*(x01 - x2) + (y01 - y2)*(y01 - y2)) < nThresSet)
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
						if (vcPointAlias[k].angle < dAngSt || vcPointAlias[k].angle > dAngEnd)
							continue;						
					}
					else
					{
						if (vcPointAlias[k].angle > dAngSt && vcPointAlias[k].angle < dAngEnd)
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
							if (vcPointAlias[j].angle < dAngSt || vcPointAlias[j].angle > dAngEnd)
								continue;						
						}
						else
						{
							if (vcPointAlias[j].angle > dAngSt && vcPointAlias[j].angle < dAngEnd)
								continue;							
						}
					}	
               //#endif			
					double x2 = vcPointAlias[j].x;
					double y2 = vcPointAlias[j].y;
					
					if (vcPointAlias[k].dist == vcPointAlias[j].dist  && vcPointAlias[k].angle == vcPointAlias[j].angle)  //seems go for nothing	   
						continue;				
			
					if (((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2)) < nThresSet  || ((x01 - x2)*(x01 - x2) + (y01 - y2)*(y01 - y2)) < nThresSet)
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
					if (vcPointAlias[k].angle < dAngSt || vcPointAlias[k].angle > dAngEnd)
						continue;						
				}
				else
				{
					if (vcPointAlias[k].angle > dAngSt && vcPointAlias[k].angle < dAngEnd)
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
						if (vcPointAlias[j].angle < dAngSt || vcPointAlias[j].angle > dAngEnd)
							continue;						
					}
					else
					{
						if (vcPointAlias[j].angle > dAngSt && vcPointAlias[j].angle < dAngEnd)
							continue;							
					}
				}	
   			//#endif			
				double x2 = vcPointAlias[j].x;
				double y2 = vcPointAlias[j].y;
				
				if (vcPointAlias[k].dist == vcPointAlias[j].dist  && vcPointAlias[k].angle == vcPointAlias[j].angle)  //seems go for nothing	   
					continue;				
		
				if (((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2)) < nThresSet  || ((x01 - x2)*(x01 - x2) + (y01 - y2)*(y01 - y2)) < nThresSet)
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
			if (vcPointGet[i].angle == vcPoint[j].dAngle && vcPointGet[i].dist == vcPoint[j].u16Dist)
			{
				vcPoint.erase(vcPoint.begin()+j);
				break;
			}
		}
#if EN_INHERIT==1
		for( k = 0; k < vcPointData.size(); k++)
		{
			if(vcPointGet[i].angle == vcPointData[k].angle && vcPointGet[i].dist == vcPointData[k].dist)
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
void pointCloudFilters::GetDistDotThres(unsigned int nDist,double &dDistOfPtFac, double &dCaliFac, int &nThresSet)
{

#ifdef EN_THRES_CALIBRTE
	if (nDist)  // 
	{
		if (nDist < 200)
			nThresSet = 10;
		else if (nDist < 225)
			nThresSet = 10 + SINGLE_DIST_OFST / 4;	 //C=  S=12
		else if (nDist < 250)
			nThresSet = 10 + SINGLE_DIST_OFST / 2;	 //C=  S=14 		
		else if (nDist < 275)
			nThresSet = 10 + SINGLE_DIST_OFST * 3 / 4;	//C=  S=16
		else if (nDist < 300)
			nThresSet = 10 + SINGLE_DIST_OFST;	  //C=	S=18			
		else if (nDist < 350)
			nThresSet = 10 + SINGLE_DIST_OFST + SINGLE_DIST_OFST / 2;  //C=  S=22				
		else if (nDist < 400)
			nThresSet = 10 + 3 * SINGLE_DIST_OFST;						 //C=  S=34 	
		else if (nDist < 500)
			nThresSet = 10 + 6 * SINGLE_DIST_OFST;	//C=118    S=58
		else if (nDist < 600)
			nThresSet = 10 + 10 * SINGLE_DIST_OFST; //C=118    S=90 	
		else if (nDist < 700)
			nThresSet = 10 + 14 * SINGLE_DIST_OFST; //C=118    S=122
		else if (nDist < 800)
			nThresSet = 10 + 18 * SINGLE_DIST_OFST; //C=118    S=154		
		else if (nDist < 900)
			nThresSet = 10 + 22 * SINGLE_DIST_OFST; //C=150    S=186
		else if (nDist <= 1000)
			nThresSet = 10 + 25 * SINGLE_DIST_OFST; //C=185.23 S=210		
	}

	nThresSet = nThresSet * (dCaliFac);

#else
	if (nDist)
	{
		nThresSet = nDist * dDistOfPtFac;
		nThresSet = nThresSet * nThresSet;
		nThresSet = nThresSet * SINGLE_PT_FAC;
		nThresSet = nThresSet * (dCaliFac);
	}
#endif

}

/**************************************************************************************************************************/
//condition Comment: dAngSt<=dAngEnd
/**************************************************************************************************************************/
bool pointCloudFilters::angleInRange(const double dAngleIn, const double &dAngSt , const double &dAngEnd )
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
// Not use
/**************************************************************************************************************************/
void pointCloudFilters::ScopeSeek(LstPointCloud &vcPoint,double &dAngSt, double &dAngEnd,bool bEnClrAng)
{		
	return; 		
}


