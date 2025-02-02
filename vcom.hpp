#ifndef _NEONLICHT_VCOM_H
#define _NEONLICHT_VCOM_H

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <usb.h>
#include <string>
#include <filesystem>
#include <fstream>
#include <algorithm>
#include <thread>
#include <chrono>
#include <math.h>
#include "procfs.hpp"

/*TODO: 
заполнить енамы, исправить логику работы порта на стороне МК
протестить открытие порта и отправку данных
написать асинки
протестить прием данных
упаковать все это в классы*/

enum class alignas(8) CMD
{
    
};

enum class alignas(8) MODE
{

};

constexpr int NEONLICHT_PID = 0x56CD;
constexpr int NEONLICHT_VID = 0x0483;

int serial_port = 0;
std::string devPath;
termios tty;

void vcpInit()
{
    usb_init();
}

bool deviceConnected()
{
    struct usb_bus* bus;
    struct usb_device* dev;
    
    usb_find_busses();
    usb_find_devices();
    for(bus = usb_busses; bus; bus = bus->next)
    {
        for(dev = bus->devices; dev; dev = dev->next)
        {
            if(dev->descriptor.idProduct == NEONLICHT_PID && dev->descriptor.idVendor == NEONLICHT_VID)
            {
                return true;
            }
        }
    }

    return false;
}

std::string findDevPath()
{
    const std::filesystem::path sysPath{"/sys/class/tty/"};
    std::filesystem::path ttyPath;
    std::string devicePath = "";
    bool isFound = false;
    for(auto& tty : std::filesystem::directory_iterator{sysPath})
    {
        if(tty.is_directory() && std::filesystem::exists(std::string{tty.path()} + "/device/uevent"))
        {
            /*read tty/device/uevent file with PID and VID*/
            std::fstream ueventFileDevice{std::string{tty.path()} + "/device/uevent", std::ios::in};
            if(ueventFileDevice.is_open())
            {
                std::string str;
                while(std::getline(ueventFileDevice, str))
                {
                    size_t pos = str.find("PRODUCT=");
                    if(pos != std::string::npos)
                    {
                        pos = str.find("=") + 1;
                        str = str.substr(pos, str.length() - pos);
                        if(str == "483/56cd/200")
                        {
                            isFound = true;
                            break;
                        }
                    }
                }

                if(isFound)
                {
                    ttyPath = tty;
                    break;
                }
            }
            ueventFileDevice.close();
        }
    }

    /*read tty uevent file*/
    std::fstream ueventFileTTY{std::string{ttyPath} + "/uevent", std::ios::in};
    if(ueventFileTTY.is_open() && isFound)
    {
        std::string str;
        while(std::getline(ueventFileTTY, str))
        {
            size_t pos = str.find("DEVNAME=");
            if(pos != std::string::npos)
                break;
        }
        size_t pos = str.find("=") + 1;
        devicePath = "/dev/" + str.substr(pos, str.length() - pos);

        ueventFileTTY.close();
    }

    return devicePath;
}

void openPort()
{
    serial_port = open(devPath.c_str(), O_RDWR);

    if(serial_port < 0)
    {
        std::cerr << "Errot " << errno << " from open: " << strerror(errno) << std::endl;
    }

    if(tcgetattr(serial_port, &tty) != 0)
    {
        std::cerr << "Error " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
    }

    /*now set setting for the serial port*/
    /*!!! BYTEFUCKING WARNING !!!*/
    /*c_cflags - CONTROL MODE*/
    tty.c_cflag &= ~PARENB;     /*Parity - disabled*/
    tty.c_cflag &= ~CSTOPB;     /*Num of stop bits - 1*/
    tty.c_cflag |=  CS8;        /*Num of bits per byte - 8*/
    tty.c_cflag &= ~CRTSCTS;    /*Hardware flow control(RTS/CTS) - disabled*/
    tty.c_cflag |=  CREAD | CLOCAL;     /*Turn on read and ignore ctrl lines*/

    /*c_lflag - LOCAL MODES*/
    tty.c_lflag &= ~ICANON;     /*Canonical mode - disabled*/
    tty.c_lflag &= ~ECHO;       /*Echo - all disabled because canon. mode is disabled*/
    tty.c_lflag &= ~ECHOE;
    tty.c_lflag &= ~ECHONL;
    tty.c_lflag &= ~ISIG;       /*Signal chars - disabled*/

    /*c_iflag - INPUT MODES*/
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);     /*Software flow control - disabled*/
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);    /*Special handling for input - disabled*/

    /*c_oflags - OUTPUT MODES*/
    tty.c_oflag &= ~OPOST;      /*Special handling for output - disabled*/
    tty.c_oflag &= ~ONLCR;      /*Conversion of new line char into CRLF*/

    /*c_cc - VMIN&VTIME settings*/
    tty.c_cc[VMIN]  = 16;    /*wait 16 bytes before returning*/
    tty.c_cc[VTIME] = 0;    /*block indefinitely*/

    /*set baud rate 115200*/
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);

    /*save these new settings*/
    if(tcsetattr(serial_port, TCSANOW, &tty) != 0)
    {
        std::cerr << "Error " << errno << " in tcsetattr: " << strerror(errno) << std::endl;
    }
}

void closePort()
{
    if(serial_port > 0)
    {
        close(serial_port);
    }
}

std::string getRAM()
{
    pfs::procfs proc;
    auto mem = proc.get_meminfo();
    double memTotal, memAvailable;
    int ramUsed;
    
    auto memItr = mem.find("MemAvailable");
    if(memItr == mem.end())
        return "0";
    memAvailable = (*memItr).second;

    memItr = mem.find("MemTotal");
    if(memItr == mem.end())
        return "0";
    memTotal = (*memItr).second;

    ramUsed = (memTotal - memAvailable) / 1024;

    return std::to_string(ramUsed) + "MB";
}

/*подбросок*/
void getCPU(double& percent, const int poll_ms)
{
    pfs::procfs proc;
    auto cpu = proc.get_stat();

    cpu = proc.get_stat();
    auto cpuUsage = cpu.cpus.total;
    unsigned long long prevIdle = cpuUsage.idle + cpuUsage.iowait;
    unsigned long long prevNonIdle = cpuUsage.user + cpuUsage.nice + cpuUsage.system + cpuUsage.irq + cpuUsage.softirq + cpuUsage.steal;
    unsigned long long prevTotal = prevIdle + prevNonIdle;

    std::this_thread::sleep_for(std::chrono::milliseconds{poll_ms});
    
    cpu = proc.get_stat();
    cpuUsage = cpu.cpus.total;

    unsigned long long idle = cpuUsage.idle + cpuUsage.iowait;
    unsigned long long nonIdle = cpuUsage.user + cpuUsage.nice + cpuUsage.system + cpuUsage.irq + cpuUsage.softirq + cpuUsage.steal;
    unsigned long long total = idle + nonIdle;
    unsigned long long totalD = total - prevTotal;
    unsigned long long idleD = idle - prevIdle;

    if(totalD != 0)
    {
        double cpu_precentage = (totalD - idleD) / static_cast<double>(totalD) * 100;
        percent = std::round(cpu_precentage * 10) / 10; 
    }
}

std::string getTimeString()
{
    std::time_t ctime = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::tm* cur_time = std::localtime(&ctime);

    return std::string{std::to_string(cur_time->tm_hour) + "." + std::to_string(cur_time->tm_min)};
}

std::string getDateDMString()
{
    std::time_t ctime = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::tm* cur_time = std::localtime(&ctime);

    if((cur_time->tm_mon + 1) > 10)
        return std::string{std::to_string(cur_time->tm_mday) + "." + std::to_string(cur_time->tm_mon + 1)};
    else
        return std::string{std::to_string(cur_time->tm_mday) + ".0" + std::to_string(cur_time->tm_mon + 1)};
}

std::string getDateYString()
{
    std::time_t ctime = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::tm* cur_time = std::localtime(&ctime);

    return std::string{std::to_string(cur_time->tm_year + 1900)};
}

void setMode(MODE* mode)
{
    write(serial_port, reinterpret_cast<const void*>(mode), sizeof(unsigned char));
}

void sendData(std::string data)
{
    if(data.length() <= 16 && data.at(data.length() -1) == 0)
    {
        write(serial_port, reinterpret_cast<const void*>(data.c_str()), data.length());
    }
}

/*I don't know how to implement asyncs yet >_<*/
MODE getModeAsync()
{
    
}

std::string getDataAsync()
{

}

#endif