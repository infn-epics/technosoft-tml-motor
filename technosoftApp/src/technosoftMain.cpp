/* technosoftMain.cpp — standard EPICS IOC main */

#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <cstdio>

#include "epicsExit.h"
#include "epicsThread.h"
#include "iocsh.h"

int main(int argc, char *argv[])
{
    if (argc >= 2) {
        iocsh(argv[1]);
        epicsThreadSleep(0.2);
    }
    iocsh(NULL);
    epicsExit(0);
    return 0;
}
