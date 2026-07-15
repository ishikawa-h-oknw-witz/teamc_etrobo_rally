#ifndef LOGGER_H
#define LOGGER_H

#include <cstdarg>
#include <cstdio>

#include <syssvc/serial.h>
#include <spike/hub/bluetooth.h>

class Logger
{
public:
    static void init();
    static void printf(const char* fmt, ...);
};

#endif