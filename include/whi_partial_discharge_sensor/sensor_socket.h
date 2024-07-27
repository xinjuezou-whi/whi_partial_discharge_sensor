/******************************************************************
Bingo instance of partial discharge sensor

Features:
- xxx

Written by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2024-07-03: Initial version
2024-xx-xx: xxx
******************************************************************/
#pragma once
#include "sensor_base.h"
#include "sockpp/tcp_connector.h"

#include <string>
#include <memory>

namespace whi_partial_discharge_sensor
{
    class SocketSensor : public BaseSensor
    {
    public:
        SocketSensor() = default;
        SocketSensor(const std::string& Addr, int Port);
        virtual ~SocketSensor();

    public:
        std::vector<uint8_t> readChannel(int ReadLength, int Channel) override;
        std::map<int, std::vector<uint8_t>> readChannels(int ReadLength) override;
        void close() override;

    protected:
        std::unique_ptr<sockpp::tcp_connector> connector_{ nullptr };
        int device_addr_{ 1 };
    };
} // namespace whi_partial_discharge_sensor
