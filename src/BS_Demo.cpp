
#include "base/hclidar.h"
#include "LidarTest.h"
#include <stdio.h>
#include <iostream>
#include <string> 
#include <sstream>
#include <algorithm>
#include <fstream>

#include "ChargingPoint.h"



std::string  g_strLidarID = "";


void sdkCallBackFunErrorCode(int iErrorCode)
{
    //std::cout << "Main: sdkCallBackFunErrorCode ErrorCode=" << iErrorCode << std::endl;
}

void sdkCallBackFunSecondInfo(tsSDKStatistic sInfo)
{
	/*printf("Main: sdkCallBackFunSecondInfo time=%lld s,points=%d,GrayBytes=%d,FPS=%d,speed=%0.2f,PPS=%d,valid=%d,invalid=%d,ErrorPacket=%d\n",
		sInfo.u64TimeStampS, sInfo.iNumPerPacket, sInfo.iGrayBytes, sInfo.u64FPS
		, sInfo.dRMS, sInfo.iPacketPerSecond, sInfo.iValid, sInfo.iInvalid
		, sInfo.u64ErrorPacketCount);*/

	std::string strFile = "FPS" + g_strLidarID.substr(32-14,14) + ".csv";
	std::ofstream outFile;
	outFile.open(strFile, std::ios::app);

	char buff[128] = { 0 };
	sprintf(buff, "%lld,%d,%d,%lld,%0.2f,%d,%d,%d,%lld\n",
		sInfo.u64TimeStampS, sInfo.iNumPerPacket, sInfo.iGrayBytes, sInfo.u64FPS
		, sInfo.dRMS,  sInfo.iValid, sInfo.iInvalid, sInfo.iPacketPerSecond
		, sInfo.u64ErrorPacketCount);

	outFile.write(buff, strlen(buff));
	outFile.close();
	
}

void sdkCallBackFunPointCloud(LstPointCloud lstG)
{
    std::cout << "Main: sdkCallBackFunPointCloud Rx Points=" << lstG.size() <<std::endl;
    for(auto sInfo : lstG)
    {
        //std::cout << "Main: Angle=" << sInfo.dAngle  << ",AngleRaw=" << sInfo.dAngleRaw << ",Dist=" << sInfo.u16Dist << std::endl;
    }
}

void sdkCallBackFunDistQ2(LstNodeDistQ2 lstG)
{
    std::cout << "Main: sdkCallBackFunDistQ2 Rx Points=" << lstG.size() <<std::endl;
    for(auto sInfo : lstG)
    {
        //std::cout << "Main: Angle=" << sInfo.angle_q6_checkbit/64.0f  << ",Dist=" << sInfo.distance_q2/4 << std::endl;
    }
}

int getPort()
{
	printf("Please select COM:\n");
	int iPort = 3;
    std::cin >> iPort;
	return iPort;
}

int getBaud()
{
	printf("Please select COM baud:\n");
	int iBaud = 115200;
	std::cin >> iBaud;
	return iBaud;
}

std::string getLidarModel()
{
	printf("Please select Lidar model:\n");
	std::string str = "X2M";
	std::cin >> str;
	std::transform(str.begin(), str.end(), str.begin(), ::toupper);
	return str;
}


void rechargingRecognize(LstPointCloud& lstG) 
{
    if(lstG.size()<600)
    	return;
    printf("----------------\n");
    std::vector<tsADIR> lstData;
    for(const auto & sInfo: lstG)
    {
        tsADIR dis;
        dis.angle = sInfo.dAngle;
        dis.distance = sInfo.u16Dist;
        dis.intensity = sInfo.u16Gray;
        dis.rpm = sInfo.u16Speed;
        if(sInfo.bValid)
            lstData.push_back(dis);
    }
    lstG.clear();




	int iCount=0;
	double dMaxConf = 0;
	double dMaxAngle=0;
	double dMaxDist=0;
	std::vector<tsBSInfo> lstBSInfo;
	findBSInfo(lstData, lstBSInfo);
	for( auto c : lstBSInfo)
    {
		if (std::isnan(c.dMx))
			continue;

            iCount++;
            double dAngle = HCHead::getAngleFromXY(c.dMx, c.dMy);
            double dDist = HCHead::getDistFromAB(c.dMx, c.dMy);

            //if(dAngle>290 ||  dAngle<250)
               // continue;
            if(c.dConfidence>dMaxConf)
            {
                dMaxConf = c.dConfidence;
                dMaxAngle = dAngle;
                dMaxDist = dDist;
            }
	    //char buff[128] = { 0 };
	    //printf("X=%0.3f,Y=%0.3f,angle=%0.3f,dist=%0.3f,Conf=%0.3f\n",c.center.x(),c.center.y(),dAngle,dDist,c.dConfidence);


            //std::cout  << "X=" << c.center.x() << ",Y=" << c.center.y() << ",angle=" << dAngle << ",dist=" << dDist << ",Conf=" << c.dConfidence<< std::endl;

    }
	if(dMaxConf>0)
	{
		printf("======================angle=%0.3f,dist=%0.3f,Conf=%0.3f\n",dMaxAngle,dMaxDist,dMaxConf);
	}

}

int main(int argc, char** argv )
{

	HCLidar& device = HCLidar::getInstance();
    int rtn = 0;

    bool bPollMode = true;
    bool bDistQ2 = false;
    bool bLoop = false;

    std::cout << "Main: SDK verion=" << device.getSDKVersion().c_str() << std::endl;

    //auto funErrorCode = std::bind(sdkCallBackFunErrorCode, std::placeholders::_1);
    //device.setCallBackFunErrorCode(funErrorCode);

    auto funSecondInfo = std::bind(sdkCallBackFunSecondInfo, std::placeholders::_1);
    device.setCallBackFunSecondInfo(funSecondInfo);

    //if(!bPollMode)//call back
    //{
    //    auto funPointCloud = std::bind(sdkCallBackFunPointCloud, std::placeholders::_1);
    //    device.setCallBackFunPointCloud(funPointCloud);

    //    auto funDistQ2 = std::bind(sdkCallBackFunDistQ2, std::placeholders::_1);
    //    device.setCallBackFunDistQ2(funDistQ2);
    //}

	
	int iPort = getPort();
	std::string strPort;
#ifdef _WIN32
	strPort = "//./com" + std::to_string(iPort);                     // For windows OS
#else
	strPort = "/dev/ttyS" + std::to_string(iPort);              // For Linux OS
#endif


	int iBaud = getBaud();

	std::string strLidarModel = getLidarModel();


	int iReadTimeoutms = 10;//

	rtn = device.initialize(strPort.c_str(), strLidarModel.c_str(), iBaud, iReadTimeoutms, bDistQ2, bLoop, bPollMode);

    if (rtn != 1)
    {
		device.unInit();
		printf("Main: Init sdk failed! port:%s\n",strPort.c_str() );

		getchar();
		exit(0);
		return 0;
        
    }

	/*if (!device.getLidarInfo())
	{
		device.unInit();
		printf("LidarTest: Get lidar info failed!\n");
		getchar();
		exit(0);
		return 0;
		
	}*/

	
	printf( "Lidar ID=%s\n" , device.getLidarID().c_str());
	printf( "Factory Info:%s\n" , device.getFactoryInfo().c_str());
	printf( "Main: Firmware ver:%s\n", device.getFirmwareVersion().c_str() );
	printf( "Main: Hardware ver:%s\n", device.getHardwareVersion().c_str());
	printf( "Main: Lidar model:%s\n" , device.getLidarModel().c_str() );

	g_strLidarID = device.getLidarID();

	int iCount = 0;

    LstPointCloud lstG;
    while (true)
    {

        if(bPollMode)
        {
            if(bDistQ2)
            {
                LstNodeDistQ2 lstG;
                device.getScanData(lstG, false);
		printf( "Main: Poll DistQ2 Rx Points=%ld\n" ,lstG.size() );
                for(auto sInfo : lstG)
                {
			//printf("Main: Angle=%0.2f,Dist=%d\n" ,(double)sInfo.angle_q6_checkbit/64.0f  , sInfo.distance_q2/4 );
                }
            }
            else
            {
                		LstPointCloud lstTemp;
				if (device.getRxPointClouds(lstTemp))
				{
					//printf("Main: Poll Rx Points=%ld\n",lstTemp.size());
					for (auto sInfo : lstTemp)
					{
						lstG.push_back(sInfo);
						//printf( "%0.4f, %d, %d, %d\n", sInfo.dAngle , sInfo.u16Dist, sInfo.u16Gray,sInfo.u16Speed);
					}
					
					rechargingRecognize(lstG);
				}
				else
				{
					int iError = device.getLastErrCode();
					if (iError != LIDAR_SUCCESS)
					{
						printf( "Main: Poll Rx Points error code=%d\n", iError );
						switch (iError)
						{
						case ERR_SHARK_MOTOR_BLOCKED:
							break;
						case ERR_SHARK_INVALID_POINTS:
							break;
						case ERR_LIDAR_SPEED_LOW:
							break;
						case ERR_LIDAR_SPEED_HIGH:
							break;
						case ERR_DISCONNECTED:
							break;
						case ERR_LIDAR_FPS_INVALID:
							break;
						default:
							break;
						}
					}
				}
				                
            }
        }
        int iSDKStatus = device.getSDKStatus();
		//printf("Main: SDK Status=%d\n" ,iSDKStatus );



        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        std::this_thread::yield();
        //printf("main....\n");
    }


	device.unInit();
    return 0;

}



