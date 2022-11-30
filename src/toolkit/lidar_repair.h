#pragma once
#ifndef HCLIDAR_R_H
#define HCLIDAR_R_H

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

#define PI  3.1415926
typedef std::vector<lidar_data_t> rpLstPointCloud;

class HCLidarRepair
{
public:

    ~HCLidarRepair() {}

#if SHARK_ENABLE
    HCLidarRepair();
#else
    static HCLidarRepair& getInstance()
    {
        static HCLidarRepair instance;  //局部静态变量
        return instance;
    }
#endif
    std::vector<int> m_nAngleStart;
    std::vector<int> m_nAngleEnd;
    rpLstPointCloud    m_lst;

    bool initialize()
	{
		return true;

	}
    bool unInit()
	{
		return true;
	}

    UINT16 RepairPointCloud(const std::vector<_lidar_data_t> data_in, std::vector<_lidar_data_t>& data_out, int dAngle[][2], int nParaNum)
    {
        m_nAngleStart.clear();
        m_nAngleEnd.clear();
        m_lst.clear();

        if (nParaNum < 1 || nParaNum > 10)
        {
            return ERR_ANGLE_PARA_NUM;
        }
        if (data_in.size() < 30)
        {
            return ERR_DATA_INPUT;
        }

        int nCnt = nParaNum;
        for (int idx = 0; idx < nCnt; idx++)
        {
            m_nAngleStart.push_back(dAngle[idx][0]);
            m_nAngleEnd.push_back(dAngle[idx][1]);
        }

        ///repair
        bool bRet = toLstPointCloud(data_in);
        if (bRet)
        {
            bool bRepairSuc = repairLstPointCloud(m_lst);

            if (bRepairSuc)
            {
                data_out.swap(m_lst);
            }
            else//fail
            { 
                m_lst.clear();
                return ERR_DATA_REPAIR;
            }
        }
        else
        {
            m_lst.clear();
            return ERR_DATA_INPUT;
        }
        m_lst.clear();
        return LIDAR_SUC;
    }
private:

#if SHARK_ENABLE

#else
    HCLidarRepair(){}
    HCLidarRepair(const HCLidarRepair& other)
    {

    }
#endif

    /////////////////////////////////////PointCloud repair
    bool toLstPointCloud(const std::vector<_lidar_data_t> data);
    bool toLidarData(std::vector<_lidar_data_t>& data);
    bool repairLstPointCloud(rpLstPointCloud& lstG);
    void getPeriodData(rpLstPointCloud& lstG, std::vector<double>& x_data, std::vector<double>& y_data, std::vector<double>& dAngle);
    bool repairDataLine(std::vector<double> x_data, std::vector<double> y_data, std::vector<double> angle_data, rpLstPointCloud& lstG);
    bool repairOneGap(std::vector<double> x_data, std::vector<double> y_data, std::vector<double> angle_data,
        int nRepairAngleStart, int nRepairAngleEnd, rpLstPointCloud& lstG, rpLstPointCloud& lstG_out);
};



#endif // HCLIDAR_H
