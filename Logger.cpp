#include "Logger.h"

void Logger::init()
{
    bool connected = false;

    serial_opn_por(2);

    while (!connected)
    {
        hub_bluetooth_is_connected(&connected);
        tslp_tsk(100 * 1000);
    }
}

void Logger::printf(const char* fmt, ...)
{
    char buffer[256];

    va_list args;
    va_start(args, fmt);

    int len = vsnprintf(
        buffer,
        sizeof(buffer),
        fmt,
        args);

    va_end(args);

    serial_wri_dat(2, buffer, len);
}