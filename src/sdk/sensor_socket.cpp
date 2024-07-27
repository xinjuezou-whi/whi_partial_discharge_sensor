/******************************************************************
separate include file to solve struct confliction

Features:
- xxx

Written by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_partial_discharge_sensor/sensor_socket.h"
#include "whi_partial_discharge_sensor/printfColor.h"

#include <array>
#include <iostream>
#include <cmath>

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

    static float ieee754ToDecimal(uint32_t Raw)
    {
        float sign = Raw >> 31 == 0 ? 1.0 : -1.0;
        uint32_t mantissa = (Raw & 0x7FFFFF) | 0x800000;
        int32_t exp = ((Raw >> 23) & 0xFF) - 127 - 23;
        return sign * mantissa * std::pow(2.0, exp);
    }

    SocketSensor::SocketSensor(const std::string& Addr, int Port)
    {
        connector_ = std::make_unique<sockpp::tcp_connector>();
        connector_->connect(sockpp::inet_address(Addr, Port));
        connector_->read_timeout(std::chrono::seconds(5));
    }

    SocketSensor::~SocketSensor()
    {
        close();
    }

    std::vector<uint8_t> SocketSensor::readChannel(int ReadLength, int Channel)
    {
        if (connector_->is_open() && Channel < 3)
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
            auto rc = connector_->write_n(data.data(), data.size());
            if (rc.value() > 0)
            {
                uint8_t read[ReadLength] = { 0 };
                rc = connector_->read_n(read, sizeof(read));
                if (rc.value() > 0)
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
        else
        {
            printf((std::string(LIGHT_RED) + "[error]: failed to read due to " + CLEANUP).c_str());
            if (!connector_->is_open())
            {
                printf((std::string(LIGHT_RED) + "connection is down" + CLEANUP + "\n").c_str());
            }
            else if (Channel >= 3)
            {
                printf((std::string(LIGHT_RED) + "channel %d doesn't exist" + CLEANUP + "\n").c_str());
            }
        }

        return std::vector<uint8_t>();
    }

    std::map<int, std::vector<uint8_t>> SocketSensor::readChannels(int ReadLength)
    {
        std::map<int, std::vector<uint8_t>> resData;
        for (int i = 0; i < 3; ++i)
        {
            resData[i] = readChannel(ReadLength, i);
        }

        return resData;
    }

    void SocketSensor::close()
    {
        connector_->close();
    }
} // namespace whi_partial_discharge_sensor
