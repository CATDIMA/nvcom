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

#endif