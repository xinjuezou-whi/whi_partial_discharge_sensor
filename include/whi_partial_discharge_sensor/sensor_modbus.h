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

Changelog:
2024-07-03: Initial version
2024-xx-xx: xxx
******************************************************************/
#pragma once
#include "sensor_base.h"
#include <modbus/modbus.h>

#include <string>

namespace whi_partial_discharge_sensor
{
    class ModbusSensor : public BaseSensor
    {
    public:
        ModbusSensor() = default;
        ModbusSensor(const std::string& Addr, int Port, int DeviceAddr);
        virtual ~ModbusSensor();

    public:
        std::vector<uint8_t> readChannel(int ReadLength, int Channel) override;
        std::map<int, std::vector<uint8_t>> readChannels(int ReadLength) override;
        void close() override;

    protected:
        modbus_t* handle_{ nullptr };
        int device_addr_{ 1 };
    };
} // namespace whi_partial_discharge_sensor
