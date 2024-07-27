/******************************************************************
class of base partial discharge sensor

Features:
- virtual interfaces
- xxx

Written by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2024-07-03: Initial version
2024-xx-xx: xxx
******************************************************************/
#pragma once
#include <vector>
#include <map>

// #define DEBUG

namespace whi_partial_discharge_sensor
{
    class BaseSensor
    {
    public:
        BaseSensor() = default;
        virtual ~BaseSensor() = default;

    public:
        virtual std::vector<uint8_t> readChannel(int ReadLength, int Channel) = 0;
        virtual std::map<int, std::vector<uint8_t>> readChannels(int ReadLength) = 0;
        virtual void close() = 0;
    };
} // namespace whi_rc_bridge
