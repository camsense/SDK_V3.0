
//#include "base/hclidar.h"
#include "LidarTest.h"
#include <stdio.h>
#include <iostream>
#include <string> 
#include <sstream>
#include <algorithm>
#include <fstream>
#include <numeric>

#include "base/HcData.h"
#include "base/HcSDK.h"


#define  GRAY_COUNT_MAX       100

std::string  g_strLidarID = "";


void sdkCallBackFunErrorCode(int iErrorCode)
{
	char buff[128] = { 0 };
	sprintf(buff, "Error code=%d\n",iErrorCode);
	printf(buff);
}

void sdkCallBackFunSecondInfo(tsSDKStatistic sInfo)
{
	/*printf("Main: sdkCallBackFunSecondInfo time=%lld s,points=%d,GrayBytes=%d,FPS=%d,speed=%0.2f,PPS=%d,valid=%d,invalid=%d,ErrorPacket=%d\n",
		sInfo.u64TimeStampS, sInfo.iNumPerPacket, sInfo.iGrayBytes, sInfo.u64FPS
		, sInfo.dRMS, sInfo.iPacketPerSecond, sInfo.iValid, sInfo.iInvalid
		, sInfo.u64ErrorPacketCount);*/

	std::string strFile = "";
	if(g_strLidarID.size() > DEFAULT_ID_LEN)
		strFile = "FPS_" + g_strLidarID.substr(g_strLidarID.size() - DEFAULT_ID_LEN, DEFAULT_ID_LEN) + ".csv";
	else
		strFile = "FPS_" + g_strLidarID + ".csv";
	std::ofstream outFile;
	outFile.open(strFile, std::ios::app);

	char buff[128] = { 0 };
	sprintf(buff, "%lld,%d,%d,%lld,%0.2f,%d,%d,%d,%lld\n",
		sInfo.u64TimeStampS, sInfo.iNumPerPacket, sInfo.iGrayBytes, sInfo.u64FPS
		, sInfo.dRMS,  sInfo.iValid, sInfo.iInvalid, sInfo.iPacketPerSecond
		, sInfo.u64ErrorPacketCount);

	outFile.write(buff, strlen(buff));
	outFile.close();

	//printf(buff);
	
}

void sdkCallBackFunPointCloud(LstPointCloud lstG)
{
 
	std::string strFile = "";
	if (g_strLidarID.size() > DEFAULT_ID_LEN)
		strFile = "Raw_" + g_strLidarID.substr(g_strLidarID.size() - DEFAULT_ID_LEN, DEFAULT_ID_LEN) + ".csv";
	else
		strFile = "Raw_" + g_strLidarID + ".csv";

	std::ofstream outFile;
	outFile.open(strFile, std::ios::app);

	char buff[128] = { 0 };
    for(auto sInfo : lstG)
    {
		//if (sInfo.dAngle > 65 && sInfo.dAngle < 115)
		//if (sInfo.dAngle > 40 && sInfo.dAngle < 160)
		{
			memset(buff, 0, 128);
			sprintf(buff, "%lld,%0.3f,%0.3f,%d,%d,%d,%d\n",
				HCHead::getCurrentTimestampUs(), sInfo.dAngle, sInfo.dAngleRaw, sInfo.u16Dist, sInfo.bValid, sInfo.u16Speed, sInfo.u16Gray);

			outFile.write(buff, strlen(buff));

			printf(buff);
		}

		
    }
		
	outFile.close();

	sprintf(buff, "---------------------------------------  %d\n",lstG.size());

	printf(buff);
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


int main()
{

    //HCLidar& device= HCLidar::getInstance();
    int rtn = 0;

    bool bPollMode = false;
    bool bDistQ2 = false;
    bool bLoop = false;

	std::string strVer = getSDKVersion();
    std::cout << "Main: SDK verion=" << strVer.c_str()<< std::endl;

    auto funErrorCode = std::bind(sdkCallBackFunErrorCode, std::placeholders::_1);
	setSDKCallBackFunErrorCode(funErrorCode);

    auto funSecondInfo = std::bind(sdkCallBackFunSecondInfo, std::placeholders::_1);
    setSDKCallBackFunSecondInfo(funSecondInfo);

    if(!bPollMode)//call back
    {
        auto funPointCloud = std::bind(sdkCallBackFunPointCloud, std::placeholders::_1);
        setSDKCallBackFunPointCloud(funPointCloud);

        auto funDistQ2 = std::bind(sdkCallBackFunDistQ2, std::placeholders::_1);
        setSDKCallBackFunDistQ2(funDistQ2);
    }

	
	int iPort = getPort();
	std::string strPort;
#ifdef _WIN32
	strPort = "//./com" + std::to_string(iPort);                     // For windows OS
#else
	strPort = "/dev/ttyUSB" + std::to_string(iPort);              // For Linux OS
#endif


	int iBaud = getBaud();

	std::string strLidarModel = getLidarModel();


	int iReadTimeoutms = 10;//

	//setSDKFactoryMode();
	setSDKCircleDataMode();
	rtn = hcSDKInitialize(strPort.c_str(), strLidarModel.c_str(), iBaud, iReadTimeoutms, bDistQ2, bLoop, bPollMode);

    if (rtn != 1)
    {
		hcSDKUnInit();
		printf("Main: Init sdk failed!\n");
		getchar();
		exit(0);
		return 0;
        
    }

	std::this_thread::sleep_for(std::chrono::milliseconds(3000));
	g_strLidarID = getSDKLidarID();
	
	printf( "Lidar ID=%s\n" , getSDKLidarID());
	printf( "Factory Info:%s\n" , getSDKFactoryInfo());
	printf( "Main: Firmware ver:%s\n", getSDKFirmwareVersion() );
	printf( "Main: Hardware ver:%s\n", getSDKHardwareVersion());
	printf( "Main: Lidar model:%s\n" , getSDKLidarModel() );

	

	std::string strFile = "";
	if (g_strLidarID.size() > DEFAULT_ID_LEN)
		strFile = "Gray_" + g_strLidarID.substr(g_strLidarID.size() - DEFAULT_ID_LEN, DEFAULT_ID_LEN) + ".csv";
	else
		strFile = "Gray_" + g_strLidarID + ".csv";

	std::ofstream outFile;
	outFile.open(strFile, std::ios::app);
	char buff[128] = { 0 };


	std::vector<UINT16> lstDist;
	std::vector<UINT16> lstWhite;
	std::vector<UINT16> lstLattice;
	int iTestCount = 0;
	int iCount = 0;


    while (true)
    {

        if(bPollMode)
        {
            if(bDistQ2)
            {
                LstNodeDistQ2 lstG;
                getSDKScanData(lstG, false);
				printf( "Main: Poll DistQ2 Rx Points=%d\n" ,lstG.size() );
                for(auto sInfo : lstG)
                {
					//printf("Main: Angle=%0.2f,Dist=%d\n" ,(double)sInfo.angle_q6_checkbit/64.0f  , sInfo.distance_q2/4 );
                }
            }
            else
            {
                LstPointCloud lstG;
				if (getSDKRxPointClouds(lstG))
				{
					if (lstG.size() > 0)
					{
						//printf("Main: ------------------------------Poll Rx Points=%d\n", lstG.size());
						UINT16 u16Dist = 0;
						for (auto sInfo : lstG)
						{
							if (!sInfo.bValid)
								continue;
							//lstDist.push_back(sInfo.u16Dist);
							//printf("Main: Angle=%0.4f, Dist=%d\n", sInfo.dAngle, sInfo.u16Dist);
							
							if (sInfo.dAngle > 181 && sInfo.dAngle < 182)
							{
								u16Dist = sInfo.u16Dist;
								printf("Main: 1     Angle=%0.4f,Dist=%d,Gray=%d\n", sInfo.dAngle, sInfo.u16Dist, sInfo.u16Gray);
								lstLattice.push_back(sInfo.u16Gray);
								if (lstLattice.size() > GRAY_COUNT_MAX)
								{
									lstLattice.erase(lstLattice.begin());
								}
							}
								

							if (sInfo.dAngle > 177 && sInfo.dAngle < 178)
							{
								printf("Main: 2     Angle=%0.4f,Dist=%d,Gray=%d\n", sInfo.dAngle, sInfo.u16Dist, sInfo.u16Gray);
								lstWhite.push_back(sInfo.u16Gray);
								if (lstWhite.size() > GRAY_COUNT_MAX)
								{
									lstWhite.erase(lstWhite.begin());
								}
							}
								
							
						}
						/*
						if (lstDist.size() > 2000)
						{

							printf("-----------------------------------------------------\n");
							std::sort(lstDist.begin(), lstDist.end());
							UINT16 dDist = std::accumulate(lstDist.begin(), lstDist.end(), 0) / lstDist.size();
							printf("Main:Dist size=%d, mean=%d, median=%d\n", lstDist.size(), dDist, lstDist.at(lstDist.size() / 2));


							lstDist.clear();
						}*/

						if (lstLattice.size() >= GRAY_COUNT_MAX && lstWhite.size() >= GRAY_COUNT_MAX)
						{
							printf("-----------------------------------------------------\n");

							for (auto tmp : lstLattice)
							{
								printf("Main:Lattice=%d\n", tmp);

							}

							for (auto tmp : lstWhite)
							{
								printf("Main:White=%d\n", tmp);

							}

							printf("-----------------------------------------------------\n");
							std::sort(lstLattice.begin(), lstLattice.end());
							UINT16 dLattice = std::accumulate(lstLattice.begin(), lstLattice.end(), 0) / lstLattice.size();
							printf("Main:Lattice size=%d, mean=%d, median=%d\n", lstLattice.size(), dLattice, lstLattice.at(lstLattice.size()/2));

							std::sort(lstWhite.begin(), lstWhite.end());
							UINT16 dWhite = std::accumulate(lstWhite.begin(), lstWhite.end(), 0) / lstWhite.size();
							printf("Main:White size=%d, mean=%d, median=%d\n", lstWhite.size(), dWhite, lstWhite.at(lstWhite.size() / 2));

							int iDiff = dLattice - dWhite;
							printf("Main: diff=%d\n", iDiff);
							printf("-----------------------------------------------------\n");
							

							sprintf(buff, "%lld,%d,%d,%d,%d,%d,%d\n",
								HCHead::getCurrentTimestampUs(), u16Dist,dLattice, dWhite, iDiff, lstLattice.at(lstLattice.size() / 2), lstWhite.at(lstWhite.size() / 2));

							outFile.write(buff, strlen(buff));
							outFile.flush();

							lstWhite.clear();
							lstLattice.clear();

							iTestCount++;
							if (iTestCount > 5)
								break;
						}
					}
					
				}
				else
				{
					int iError = getSDKLastErrCode();
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
        int iSDKStatus = getSDKStatus();
		//printf("Main: SDK Status=%d\n" ,iSDKStatus );

		iCount++;
		if (iCount > 100)
		{
			iCount = 0;
			//g_strLidarID = getSDKLidarID();
			//printf("Main:Lidar ID=%s\n", g_strLidarID.c_str());
		}
		

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        std::this_thread::yield();
    }
	
	outFile.close();

	hcSDKUnInit();
    return 0;

}
