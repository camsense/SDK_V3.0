// lidar_repair.cpp : 定义静态库的函数。
//

//#include "pch.h"
//#include "framework.h"
#include "lidar_repair.h"


bool HCLidarRepair::toLstPointCloud(const std::vector<_lidar_data_t> data)
{
    m_lst.clear();
    for (auto item:data)
    {
        _lidar_data_t tmpdata;
        tmpdata.dAngle = item.dAngle;
        tmpdata.dAngle = item.dAngle;
        tmpdata.u16Dist = item.u16Dist;
        tmpdata.u64TimeStampMs = item.u64TimeStampMs;
        tmpdata.confidence = item.confidence;
        m_lst.push_back(tmpdata);
    }
    return m_lst.size() > 0;
}
bool HCLidarRepair::toLidarData(std::vector<_lidar_data_t>& data)
{
    for (auto item : m_lst)
    {
        _lidar_data_t tmpdata;
        tmpdata.dAngle = item.dAngle;
        tmpdata.u16Dist = item.u16Dist;
        tmpdata.u64TimeStampMs = item.u64TimeStampMs;
        tmpdata.confidence = item.confidence;
        data.push_back(tmpdata);
    }
    return data.size() > 0;
}
/**get_period_data
@vCmosData[in]   cmos point cloud data
@x_data[out]     x
@y_data[out]     y
@dAngle[out]     dAngle
*/
void HCLidarRepair::getPeriodData(rpLstPointCloud& lstG, std::vector<double>& x_data, std::vector<double>& y_data, std::vector<double>& dAngle)
{
    for (auto item : lstG)
    {
        dAngle.push_back(item.dAngle);
        double u16Dist = (double)item.u16Dist;
        x_data.push_back(u16Dist * cos(item.dAngle * PI / 180.0));
        y_data.push_back(u16Dist * sin(item.dAngle * PI / 180.0));
    }

}

bool HCLidarRepair::repairLstPointCloud(rpLstPointCloud& lstG)
{
    if (lstG.size() < 30)
    {
        return false;
    }

    rpLstPointCloud lstTemp;
    lstTemp.swap(lstG);

    //repair
    std::vector<double> angle_data;
    std::vector<double> x_data;
    std::vector<double> y_data;
    std::vector<double> y_data_repair;
    std::vector<double> angle_data_repair;

    getPeriodData(lstTemp, x_data, y_data, angle_data);
    bool bRet = repairDataLine(x_data, y_data, angle_data, lstTemp);//直线填补

    lstG.swap(lstTemp);
    return bRet;
}


bool HCLidarRepair::repairOneGap(std::vector<double> x_data, std::vector<double> y_data, std::vector<double> angle_data,
    int nRepairAngleStart, int nRepairAngleEnd,
    rpLstPointCloud& lstG, rpLstPointCloud& lstG_out)
{
    //std::cout << "repairOneGap.....start" <<lstG.size()<<","<<nRepairAngleStart<<","<< nRepairAngleEnd<< endl;
    std::vector<double> x_data_Repair;//缺口的数据
    std::vector<double> y_data_Repair;//缺口的数据
    std::vector<double> angle_data_Repair;//缺口的数据
    std::vector<int> index_Repair;//缺口的数据
    float mean_x_1 = 0;
    float mean_y_1 = 0;
    float mean_x_2 = 0;
    float mean_y_2 = 0;
    int nCount1 = 0;
    int nCount2 = 0;

    float mean_Dist_1 = 0;
    float mean_Dist_2 = 0;
    int nMax1 = 0;
    int nMax2 = 0;

    int nSearchLen = 4;
    for (int i = 0; i < x_data.size(); i++)
    {
        if (angle_data[i] > nRepairAngleStart - nSearchLen && angle_data[i] < nRepairAngleStart && lstG[i].u16Dist < 1000 && abs(y_data[i])>10)
        {
            mean_x_1 += x_data[i];
            mean_y_1 += y_data[i];

            mean_Dist_1 += lstG[i].u16Dist;
            nMax1 = nMax1 < lstG[i].u16Dist ? lstG[i].u16Dist : nMax1;

            nCount1++;
        }

        if (angle_data[i] > nRepairAngleEnd && angle_data[i] < nRepairAngleEnd + nSearchLen && lstG[i].u16Dist < 1000 && abs(y_data[i])>10)
        {
            mean_x_2 += x_data[i];
            mean_y_2 += y_data[i];

            mean_Dist_2 += lstG[i].u16Dist;
            nMax2 = nMax2 < lstG[i].u16Dist ? lstG[i].u16Dist : nMax2;

            nCount2++;
        }
        if (angle_data[i] > nRepairAngleStart && angle_data[i] < nRepairAngleEnd && lstG[i].u16Dist < 1000 && abs(y_data[i])>10)
        {
            x_data_Repair.push_back(x_data[i]);
            y_data_Repair.push_back(y_data[i]);
            angle_data_Repair.push_back(angle_data[i]);
            index_Repair.push_back(i);
        }
    }

    //修补缺口
    if (x_data_Repair.size() > 0 && nCount1 > 1 && nCount2 > 1)
    {
        //缺口附近突变，不补缺口
        mean_Dist_1 /= nCount1;
        mean_Dist_2 /= nCount2;
        float fRatio = 1.6;
        if (abs(mean_Dist_1) > 500 || abs(mean_Dist_2) > 500)
        {
            fRatio = 1.3;
        }
        if (nMax2 / mean_Dist_1 > fRatio || nMax1 / mean_Dist_2 > fRatio)
        {
            // std::cout << "repairOneGap....fail:" << ",AngleStart=" << nRepairAngleStart << ",AngleEnd=" << nRepairAngleEnd << mean_x_1 << "," << nMax1 << "," << mean_x_2 << "," << nMax2 << endl;
            return 0;
        }
        //

        mean_x_1 /= (nCount1 + 0.0001f);
        mean_y_1 /= (nCount1 + 0.0001f);

        mean_x_2 /= (nCount2 + 0.0001f);
        mean_y_2 /= (nCount2 + 0.0001f);

        float fK = 0;
        if (fabs(mean_x_2 - mean_x_1) > 0.00001)
        {
            fK = (mean_y_2 - mean_y_1) / (mean_x_2 - mean_x_1);
            float fB = mean_y_1 - mean_x_1 * fK;
            bool bRepairByline = true;//直接根据间距补齐

            if (fabs(fK) > 3 || bRepairByline)//>75degree
            {
                float fxEnd = mean_x_2;
                float fxStart = mean_x_1;
                int  nCnt1 = 0;
                int  nCnt2 = 0;
                float fyEnd = 0;
                float fyStart = 0;
                for (int i = 0; i < index_Repair.size(); i++)
                {
                    if (angle_data_Repair[i] - nRepairAngleEnd > 0 && angle_data_Repair[i] - nRepairAngleEnd < 2)
                    {
                        fxEnd += x_data_Repair[i];
                        fyEnd += y_data_Repair[i];
                        nCnt2++;
                    }
                    if (angle_data_Repair[i] - nRepairAngleStart < 0 && angle_data_Repair[i] - nRepairAngleStart > -2)
                    {
                        fxStart += x_data_Repair[i];
                        fyStart += y_data_Repair[i];
                        nCnt1++;
                    }
                }
                if (nCnt1 > 0 && nCnt2 > 0)
                {
                    fxStart /= nCnt2;
                    fyStart /= nCnt2;
                    fxEnd /= nCnt1;
                    fyEnd /= nCnt1;
                }
                else
                {
                    fxEnd = mean_x_2;
                    fxStart = mean_x_1;
                }
                float fxGap = (fxEnd - fxStart) / index_Repair.size();
                float fK0 = fK;
                float fB0 = fB;

                // std::cout << "repair000..." <<fxStart<<","<<fxEnd<<","<<fyStart<<","<<fyEnd<<","<<fK0<<","<<fB0<<endl;
                for (int i = 0; i < index_Repair.size(); i++)
                {
                    float fxData = fxStart + fxGap * i;
                    float y_cur = fxData * fK0 + fB0;
                    float fTheta = atan(y_cur / fxData) / PI * 180;

                    int nTheta0 = fTheta * 100;
                    nTheta0 = (nTheta0 + 36000) % 36000;
                    fTheta = nTheta0 / 100.0;

                    if (fxData < 0 && y_cur < 0)
                    {
                        fTheta = fmod(fTheta + 180, 360);
                    }
                    double dSin = sin(fTheta * PI / 180);
                    if (fabs(dSin) < 0.00001)
                        dSin = 0.00001;

                   // lstG_out[index_Repair[i]].dAngleDisp = fTheta;
                    lstG_out[index_Repair[i]].dAngle = fTheta;
                    lstG_out[index_Repair[i]].u16Dist = sqrt(fxData * fxData + y_cur * y_cur);
                }
            }
            else
            {

                for (int i = 0; i < index_Repair.size(); i++)
                {
                    lstG[index_Repair[i]].dAngle = angle_data_Repair[i];
                    float y_cur = x_data_Repair[i] * fK + fB;
                    float fTheta = atan(y_cur / x_data_Repair[i]) / PI * 180;

                    int nTheta0 = fTheta * 100;
                    nTheta0 = (nTheta0 + 36000) % 36000;
                    fTheta = nTheta0 / 100.0;

                    double dSin = sin(fTheta * PI / 180);
                    if (fabs(dSin) < 0.00001)
                        dSin = 0.00001;

                    lstG_out[index_Repair[i]].dAngle = fTheta;
                    lstG_out[index_Repair[i]].u16Dist = abs(y_cur / dSin);
                }
            }
        }

    }
    return 1;
}
/**repair_data
@x_data[in]   x
@y_data[in]     y
@y_data_repair[out]     y_repair
@angle_data_repair[out]   angle_data_repair
*/
bool HCLidarRepair::repairDataLine(std::vector<double> x_data, std::vector<double> y_data, std::vector<double> angle_data, rpLstPointCloud& lstG)
{
    if (x_data.size() == 0 || y_data.size() == 0)
    {
        return false;
    }

    if (0 == m_nAngleStart.size())
    {
        return true;
    }

    rpLstPointCloud lstG_out(lstG);
    for (int nSec = 0; nSec < m_nAngleStart.size(); nSec++)
    {
        repairOneGap(x_data, y_data, angle_data, m_nAngleStart[nSec], m_nAngleEnd[nSec], lstG, lstG_out);
    }
    lstG.swap(lstG_out);
    return true;
}

