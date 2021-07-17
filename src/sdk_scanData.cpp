#include "base/lidar.h"
#include <stdio.h>

#include <fstream>

int main(int argc, char **argv)
{
    int com_id = 0;             // Open serial port using valid COM id
#ifdef _WIN32
    com_id = 3;
#else
    com_id = 0;
#endif
    if (argc >=2)
    {
        com_id = atoi(argv[1]);
    }

    char buff[32];
#ifdef _WIN32
    sprintf(buff, "//./com%d", com_id);
#else
    sprintf(buff, "/dev/ttyUSB%d", com_id);
#endif

    Dev device;
    int rs = device.Initialize(buff, 153600, true);
    if (rs != 0)
    {
        printf("Initialize failed.\n");
        return 0;
    }

    bool isExit = false;
    printf("Press any key exit:\n");
    //press any key exit
    auto threadExitFunc = [&] {
        getchar();
        isExit = true;
    };

    std::thread thr(threadExitFunc);
    thr.detach();

    std::ofstream out;
    out.open("AllData.csv");

    //std::ofstream out1;
    //out1.open("headTailData.csv");

    bool isReverse = true;
    while (!isExit)
    {
  /*      node_info nodebuffer[2048];
        size_t count = 0;
        
        device.GetScanData(nodebuffer, 2048, count, isReverse);
        
        int errcode = device.GetLastErrCode();
        if (errcode != LIDAR_SUCCESS)
        {
            printf("errcode:%d\n", errcode);
        }
            
        if (count == 0)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        for (size_t i = 0;i < count; ++i)
        {
            char buff[128] = { 0 };
            sprintf(buff, "%.4f,%d,%d,%d\n", nodebuffer[i].angle_q6_checkbit / 64.0f, 
                nodebuffer[i].distance_q2 / 4, 
                nodebuffer[i].isValid,
                nodebuffer[i].syn_quality);
            out.write(buff, strlen(buff));
        }

        out.write("\n", 1);
        char headTail[64] = {0};
        sprintf(headTail, "%.4f,%.4f,%d\n", nodebuffer[0].angle_q6_checkbit / 64.0f, nodebuffer[count - 1].angle_q6_checkbit / 64.0f, count);
        out1.write(headTail, strlen(headTail));*/

        std::list<node_info> dataList;
        device.GetScanData(dataList, isReverse);

        int errcode = device.GetLastErrCode();
        if (errcode != LIDAR_SUCCESS)
        {
            printf("errcode:%d\n", errcode);
        }

        if (dataList.empty())
        {
            //std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        if (dataList.size() < 300)
        {
            printf("datasize = %d\n", dataList.size());
        }

        for (auto it = dataList.begin(); it != dataList.end(); ++it)
        {
            char buff[128] = { 0 };
            sprintf(buff, "%.4f,%d,%d,%d\n", it->angle_q6_checkbit / 64.0f,
                it->distance_q2 / 4,
                it->isValid,
                it->syn_quality);
            out.write(buff, strlen(buff));
        }

        out.write("\n", 1);

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    device.Uninit();

    out.close();
    //out1.close();

    return 0;
}
