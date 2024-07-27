/******************************************************************
ModBUS instance of partial discharge sensor

Features:
- ModBUS
- ModBUC TCP
- ModBUS RTU
- xxx

Written by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_partial_discharge_sensor/sensor_modbus.h"
#include "whi_partial_discharge_sensor/printfColor.h"

#include <array>
#include <iostream>>

namespace whi_partial_discharge_sensor
{
    static uint16_t crc16(const uint8_t* Data, size_t Length)
    {
        uint16_t crc = 0xffff;
        uint16_t polynomial = 0xa001;

        for (size_t i = 0; i < Length; ++i)
        {
            crc ^= Data[i];
            for (int j = 0; j < 8; ++j)
            {
                if ((crc & 0x0001))
                {
                    crc = (crc >> 1) ^ polynomial;
                }
                else
                {
                    crc >>= 1;
                }
            }
        }

        return crc;
    }

    ModbusSensor::ModbusSensor(const std::string& Addr, int Port, int DeviceAddr)
        : device_addr_(DeviceAddr)
    {
        handle_ = modbus_new_tcp(Addr.c_str(), Port);
        modbus_set_error_recovery(handle_, modbus_error_recovery_mode(
            modbus_error_recovery_mode::MODBUS_ERROR_RECOVERY_LINK |
            modbus_error_recovery_mode::MODBUS_ERROR_RECOVERY_PROTOCOL));
        uint32_t oldResponseToSec;
        uint32_t oldResponseToUsec;
        modbus_get_response_timeout(handle_, &oldResponseToSec, &oldResponseToUsec);
        if (modbus_connect(handle_) == -1)
        {
            close();
            printf((std::string(LIGHT_RED) + "[error]: failed to connect ModbusTCP on %s:%d" + CLEANUP + "\n").c_str(),
                Addr.c_str(), Port);
        }
        uint32_t newResponseToSec;
        uint32_t newResponseToUsec;
        modbus_get_response_timeout(handle_, &newResponseToSec, &newResponseToUsec);
        if (oldResponseToSec != newResponseToSec || oldResponseToUsec != newResponseToUsec)
        {
            printf((std::string(LIGHT_RED) + "[error]: there is response timeout modification on connect" + CLEANUP + "\n").c_str());
        }
    }

    ModbusSensor::~ModbusSensor()
    {
        close();
    }

    std::vector<uint8_t> ModbusSensor::readChannel(int ReadLength, int Channel)
    {
        if (handle_ != nullptr)
        {
            std::array<uint8_t, 8> data{ 0x01, 0x03, 0x00, uint8_t(Channel), 0x00, 0x04, 0x00, 0x00 };
            uint16_t crc = crc16(data.data(), data.size() - 2);
            data[6] = uint8_t(crc);
            data[7] = uint8_t(crc >> 8);
#ifdef DEBUG
            std::cout << "writing data:" << std::endl;
            for (const auto& it : data)
            {
                std::cout << std::hex << int(it) << ",";
            }
            std::cout << std::dec << " with size " << data.size() << std::endl;
#endif
            int rc = modbus_write_bits(handle_, device_addr_, data.size(), data.data());
            if (rc > 0)
            {
                uint8_t read[ReadLength] = { 0 };
                rc = modbus_read_bits(handle_, device_addr_, sizeof(read), read);
                if (rc > 0)
                {
#ifdef DEBUG
                    std::cout << "reading data:" << std::endl;
                    for (const auto& it : read)
                    {
                        std::cout << std::hex << int(it) << ",";
                    }
                    std::cout << std::dec << " with size " << sizeof(read) << std::endl;
#endif
                    crc = crc16(read, ReadLength - 2);
                    if (uint8_t(crc) == read[ReadLength - 2] && uint8_t(crc >> 8) == read[ReadLength - 1])
                    {
                        std::vector<uint8_t> resData;
                        for (const auto& it : read)
                        {
                            resData.push_back(it);
                        }

                        return resData;
                    }
                    else
                    {
                        printf((std::string(YELLOW) + "[warn]: got mismet crc code\n" + CLEANUP).c_str());
                        return std::vector<uint8_t>();
                    }
                }
                else
                {
                    printf((std::string(LIGHT_RED) + "[error]: failed to read\n" + CLEANUP).c_str());
                }
            }
            else
            {
                printf((std::string(LIGHT_RED) + "[error]: failed to write\n" + CLEANUP).c_str());
            }
        }

        return std::vector<uint8_t>();
    }

    std::map<int, std::vector<uint8_t>> ModbusSensor::readChannels(int ReadLength)
    {
        std::map<int, std::vector<uint8_t>> resData;
        for (int i = 0; i < 3; ++i)
        {
            resData[i] = readChannel(ReadLength, i);
        }

        return resData;
    }

    void ModbusSensor::close()
    {
        modbus_close(handle_);
        modbus_free(handle_);
        handle_ = nullptr;
    }
} // namespace whi_partial_discharge_sensor
