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
#include "procfs.hpp"

/*TODO: упаковать все это в класс*/
/*      там функция getCPU есть, это пиздец*/

int serial_port = 0;
std::string devPath;
constexpr int NEONLICHT_PID = 0x56CD;
constexpr int NEONLICHT_VID = 0x0483;

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
        printf("Error %i from open %s\n", errno, strerror(errno));
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
void getCPU()
{
    pfs::procfs proc;
    auto cpu = proc.get_stat();

    cpu = proc.get_stat();
    auto cpuUsage = cpu.cpus.total;
    unsigned long long prevIdle = cpuUsage.idle + cpuUsage.iowait;
    unsigned long long prevNonIdle = cpuUsage.user + cpuUsage.nice + cpuUsage.system + cpuUsage.irq + cpuUsage.softirq + cpuUsage.steal;
    unsigned long long prevTotal = prevIdle + prevNonIdle;

    std::chrono::steady_clock::time_point currentTime = std::chrono::steady_clock::now();
    std::chrono::steady_clock::duration pollingPeriod = std::chrono::seconds{1};

    for(int i = 0; i < 10;)
    {
        if((std::chrono::steady_clock::now() - currentTime) >= pollingPeriod)
        {
            cpu = proc.get_stat();
            cpuUsage = cpu.cpus.total;
            currentTime = std::chrono::steady_clock::now();
            unsigned long long idle = cpuUsage.idle + cpuUsage.iowait;
            unsigned long long nonIdle = cpuUsage.user + cpuUsage.nice + cpuUsage.system + cpuUsage.irq + cpuUsage.softirq + cpuUsage.steal;
            unsigned long long total = idle + nonIdle;

            unsigned long long totalD = total - prevTotal;
            unsigned long long idleD = idle - prevIdle;

            if(totalD != 0)
            {
                double cpu_precentage = (totalD - idleD) / static_cast<double>(totalD) * 100;
                int cpu_precentage_int = static_cast<int>(cpu_precentage);
                std::cout << cpu_precentage_int<< "%" << std::endl;
            }

            prevIdle = cpuUsage.idle + cpuUsage.iowait;
            prevNonIdle = cpuUsage.user + cpuUsage.nice + cpuUsage.system + cpuUsage.irq + cpuUsage.softirq + cpuUsage.steal;
            prevTotal = prevIdle + prevNonIdle;
            ++i;
        }
    }
}

#endif