#include "base/ReadParsePackage.h"
#include <stdio.h>

int main()
{
    const int data_num = 8;
    rangedata dataPack[data_num];
    ReadParsePackage device;
    int fps = 0, rtn = 0;

    // ##### 1. Open serial port using valid COM id #####
#ifdef _WIN32
    rtn = device.OpenSerial("//./com3", 115200);                       // For windows OS
#else
    rtn = device.OpenSerial("/dev/ttyUSB0", 115200);                // For Linux OS
#endif

    if (rtn != 1)
    {
        printf("Error: Unable to open serial port!\n");
        getchar();
        exit(0);
    }

    while (true)
    {
        // ##### 2. Read data from serial port #####
        rtn = device.ReadDataSerial();
        if (rtn <= 0)
        {
            printf("Warning: Unable to read serial data!\n");
            continue;
        }

        // ##### 3. Parse data #####
        device.ParseDataSerial(dataPack, fps);
        for (int i = 0; i < data_num; ++i)
        {
            printf("%d, dist: %d, angle: %f, is_invalid: %d\n",
                i, dataPack[i].dist, dataPack[i].angle, dataPack[i].flag);
        }
        printf("\n");
    }

    // ##### 4. Close serial #####
    device.CloseSerial();
    return 0;

}
