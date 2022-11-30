#pragma once
#include "HcData.h"
#include <mutex>
using namespace std;

//work space whin the module
struct Point2D
{
	double 			x;
	double 		    y;	
	unsigned int    dist;	   
	double          angle;	
	int             nfound;  	   
};
 
#if 0
typedef unsigned short             UINT16;
typedef unsigned long long int     UINT64; 

typedef struct tsPointCloud
{ 
	bool           bValid;       // true Valid point, false  invalid            
	double         dAngle;       // degree     
	UINT16         u16Dist;      // compensate distance  ,mm  
	UINT16         u16Gray;      // luminance
	bool           bGrayTwoByte; // true  u16Gray 2byte ,false u16Gray 1byte 
	UINT64         u64TimeStampMs;    // timestamp ,ms  
	float          fTemperature;  
	tsPointCloud() :
		bValid(true),
		dAngle(0.),
		u16Dist(0),
		u16DistRaw(0),
		u16Gray(0),
		bGrayTwoByte(false),
		u64TimeStampMs(0),
		fTemperature(0)
	{}
}tsPointCloud;
typedef std::vector<tsPointCloud> LstPointCloud;
#endif

class pointCloudFilters
{
public:
	LstPointCloud hotPixelFilter(LstPointCloud& lstPointCloud,int giFilterSpeed, int giFilterFPS);
	LstPointCloud hotpixelFilterDo(LstPointCloud& lstPointCloud,int giFilterSpeed, int giFilterFPS);
	void hotPixelCheck(LstPointCloud lstPointCloud, int nWkType, double dAngSt, double dAngEnd, int giFilterSpeed, int giFilterFPS,float fFacAdjHSpd,float fFacAdjLSpd);
	LstPointCloud strongLightFilterDo(LstPointCloud& lstPointCloud,  int giFilterSpeed, int giFilterFPS);
	
private:
	void checkPoint(LstPointCloud lstPointCloud, int nWkType, double dAngSt, double dAngEnd, int giFilterSpeed, int giFilterFPS,float fFacAdjHSpd,float fFacAdjLSpd);
	void ScopeSeek(LstPointCloud &vcPoint,double &dAngSt, double &dAngEnd,bool bEnClrAng);
	void GetDistDotThres(unsigned int nDist,double &dDistOfPtFac, double &dCaliFac, int &nThresSet);
	
	void FltL(LstPointCloud &vcPoint,vector<Point2D> &vcPointData,double &dAngSt, double &dAngEnd);
	void Flt1Pt(LstPointCloud &vcPoint,vector<Point2D> &vcPointData,double &dAngSt, double &dAngEnd, int giFilterSpeed,int giFilterFPS,float fFacAdj);
	void Flt2Pt(LstPointCloud &vcPoint,vector<Point2D> &vcPointData,double &dAngSt, double &dAngEnd, int giFilterSpeed,int giFilterFPS,float fFacAdj,bool bEnAngLmt);	
	
	 void FltNPt(LstPointCloud &vcPoint,vector<Point2D> &vcPointData,double &dAngSt, double &dAngEnd, int giFilterSpeed,int giFilterFPS,float fFacAdj,int nTrace,bool bEnAngLmt);
	 bool angleInRange(const double dAngleIn, const double &dAngSt , const double &dAngEnd );
	 LstPointCloud      m_pointCloud;
	 mutex              m_mutex;
};

