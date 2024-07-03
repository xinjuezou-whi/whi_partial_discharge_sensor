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
        std::vector<uint8_t> readChannel(int Channel) override;
        std::map<int, std::vector<uint8_t>> readChannels() override;
        void close() override;

    protected:
        modbus_t* handle_{ nullptr };
        int device_addr_{ 1 };
        // std::thread th_read_;
	    // std::atomic_bool terminated_{ false };
        // uint8_t pre_byte_{ SbusData::FOOTER };
        // uint8_t msg_buf_[SbusData::PAYLOAD_LEN + SbusData::HEADER_LEN + SbusData::FOOTER_LEN];
        // int index_{ 0 };
        // SbusData data_;
        // std::mutex mtx_;
    };
} // namespace whi_partial_discharge_sensor
