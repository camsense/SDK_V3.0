
//#include "base/hclidar.h"
#include "LidarTest.h"
#include <stdio.h>
#include <iostream>
#include <string> 
#include <sstream>
#include <algorithm>
#include <fstream>

#include "base/HcData.h"
#include "base/HcSDK.h"
#include "toolkit/Hctoolkit.h"

std::string  g_strLidarID = "";


#define  NOISE_FILTER_DEMO      1


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

	printf(buff);
	
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
	sprintf(buff, "---------------Size=%d\n", lstG.size());
	printf(buff);

    for(auto sInfo : lstG)
    {
		//if (sInfo.dAngle > 310 || sInfo.dAngle < 50)
		//{

			memset(buff, 0, 128);
			sprintf(buff, "%lld,%0.3f,%0.3f,%d,%d,%d,%d\n",
				sInfo.u64TimeStampNs, sInfo.dAngle, sInfo.dAngleRaw, sInfo.u16Dist, sInfo.bValid, sInfo.u16Speed, sInfo.u16Gray);

			outFile.write(buff, strlen(buff));

			//printf(buff);
		//}

		
    }
		
	outFile.close();
	
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

void readFileOfIfstream(LstPointCloud& lstG)
{
	
	std::ifstream csv_file("RawData.ini", std::ios::in);
	std::string line;

	if (!csv_file.is_open())
	{
		std::cout << "Error: opening file fail" << std::endl;
		return;//std::exit(1);
	}

	std::istringstream sin;         //将整行字符串line读入到字符串istringstream中
	std::vector<std::string> words; //声明一个字符串向量
	std::string word;

	// 读取标题行
	//std::getline(csv_data, line);
	// 读取数据
	while (std::getline(csv_file, line))
	{
		sin.clear();
		sin.str(line);
		words.clear();
		//while (std::getline(sin, word, ',')) 
		while (std::getline(sin, word, ' ')) 
		{
			words.push_back(word); 
			//std::cout << word;
			// std::cout << atol(word.c_str());
		}
		if (words.size() >= 2)
		{
			tsPointCloud sInfo;
			sInfo.bValid = true;
			std::string str = words.at(0);
			sInfo.dAngle = atof(str.c_str());

			str = words.at(1);
			sInfo.u16Dist = (UINT16)atof(str.c_str());
			lstG.push_back(sInfo);
		}
		


		std::cout << std::endl;
		// do something。。。
	}
	csv_file.close();

}
bool filterPointCloud(LstPointCloud& lstG,const char* pModel)
{
	int rtn = 0;

	if (lstG.size() == 0)
		return false;

	if (pModel == nullptr)
		return false;

	//If need Filter 
	hcSDKFltInitialize();
	//printf("hcSDKFltInitialize() complete\n");

	//To update lidar paras of model "X2B" 
	stFltLidarCfg_t stLidarPara;
	bool bLowSpinSpeed = false;
	if (!UpdateLidarPara(pModel, bLowSpinSpeed, stLidarPara))
	{
		printf("The stLidar Para with default value\n");
		return false;
	}

	UINT64 u64Start = HCHead::getCurrentTimestampUs();

	int iInputSize = lstG.size();
	//The tsPtClouds of 1-circle Point Clouds to be processed  	
	std::vector<stPtCloud_t> tsPtClouds;
	
	for (auto& sInfo : lstG)
	{
		stPtCloud_t sTemp;
		memcpy(&sTemp, &sInfo, sizeof(stPtCloud_t));
		tsPtClouds.push_back(sTemp);

		std::cout << sInfo.dAngle << "," << sInfo.u16Dist << std::endl;
	}
	//printf("‘tsPtClouds’ size is  %d \n", tsPtClouds.size());
	std::cout <<  "-------------------------" << std::endl;

	//Can Overwrite stFltGblSetting or stFltLParas here, 
	//or with the default value which defined with micro-Define in HcPointCloudData.h
	stFltGblSetting_t stFltGblSetting;
	stFltLParas_t stFltLParas;
	stFltGblSetting.fFacAdjHSpd = 3.0;
	stFltLParas.nActDist = 800;

	//Call Filter function
	hotPixelFilter(tsPtClouds, stLidarPara, stFltGblSetting, stFltLParas);

	lstG.clear();
	lstG.resize(0);
	for (auto& sInfo : tsPtClouds)
	{
		tsPointCloud sTemp;
		memcpy(&sTemp, &sInfo, sizeof(stPtCloud_t));
		lstG.push_back(sTemp);

		std::cout << sInfo.dAngle << "," << sInfo.u16Dist << std::endl;
	}

	int iOutSize = lstG.size();

	printf("Input size=%d ,out size=%d \n", iInputSize,iOutSize);

	//Close modoule  
	hcSDKFltUnInit();

	UINT64 u64End = HCHead::getCurrentTimestampUs();

	printf("Input size=%d ,out size=%d, time = %lld us\n", iInputSize, iOutSize, (u64End- u64Start));

	return true;
}

int main()
{

#if NOISE_FILTER_DEMO
	LstPointCloud lstTemp;
	readFileOfIfstream(lstTemp);
	filterPointCloud(lstTemp, "X2A");

	return 0;
#endif


    int rtn = 0;

    bool bPollMode = false;//点云获取分轮询模式、回调模式
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


	int iReadTimeoutms = 2;//读取串口数据超时

	setSDKAngOffset(true);//启用零度角修正，需要配合雷达上电获取属性包，部分型号支持（X1S，D2系列）。
	setSDKCircleDataMode();//按圈获取点云
	rtn = hcSDKInitialize(strPort.c_str(), strLidarModel.c_str(), iBaud, iReadTimeoutms, false, bLoop, bPollMode);

    if (rtn != 1)
    {
		hcSDKUnInit();
		printf("Main: Init sdk failed!\n");
		getchar();
		exit(0);
		return 0;
        
    }

	setSDKLidarPowerOn(true);//通知camsense SDK 雷达已经上电
	setSDKPointCloudLattice(true);

	if (strLidarModel == "X2MF")
	{
		setSDKLidarLowSpeed(true);
	}


	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	/*if (!getSDKLidarInfo())
	{
		hcSDKUnInit();
		printf("Main: No lidar ID, please try again!\n");
		getchar();
		exit(0);
		return 0;
	}*/
	g_strLidarID = getSDKLidarID();
	
	
	printf( "Lidar ID=%s\n" , getSDKLidarID());
	printf( "Factory Info:%s\n" , getSDKFactoryInfo());
	printf( "Main: Firmware ver:%s\n", getSDKFirmwareVersion() );
	printf( "Main: Hardware ver:%s\n", getSDKHardwareVersion());
	printf( "Main: Lidar model:%s\n" , getSDKLidarModel() );

	

	int iCount = 0;
	//LidarTest  *lidarTest = nullptr;
    while (true)
    {

        if(bPollMode)
        {
			LstPointCloud lstG;

			if (getSDKRxPointClouds(lstG))
			{

				if (lstG.size() > 0)
				{
					//过滤杂点、射线
					//filterPointCloud(lstG, strLidarModel.c_str());

					sdkCallBackFunPointCloud(lstG);

				}

			}

			int iError = getSDKLastErrCode();
			if (iError != LIDAR_SUCCESS)
			{
				printf("Main: Poll Rx Points error code=%d\n", iError);
				switch (iError)
				{
				case ERR_SHARK_MOTOR_BLOCKED://堵转消息
					break;
				case ERR_SHARK_INVALID_POINTS://雷达被遮挡
					break;
				case ERR_DISCONNECTED://连接丢失
					break;
				case ERR_RX_CONTINUE://持续接收校验错误包
					break;
				case ERR_LIDAR_FPS_INVALID:
					break;
				default:
					break;
				}
			}
        }


        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        std::this_thread::yield();
    }

	/*if (lidarTest) 
	{
		delete lidarTest;
		lidarTest = nullptr;
	}*/

	hcSDKUnInit();
    return 0;

}
