/******************************************************************
class of partial discharge sensor

Features:
- ModbusTCP
- xxx

Written by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2024-07-03: Initial version
2023-xx-xx: xxx
******************************************************************/
#pragma once
#include <ros/ros.h>

#include "whi_partial_discharge_sensor/sensor_base.h"
#include "whi_interfaces/WhiSrvPartialDischarge.h"

namespace whi_partial_discharge_sensor
{
	class PartialDischarge
	{
    public:
        enum Hardware { HARDWARE_MODBUS_TCP = 0, HARDWARE_SOCKET, HARDWARE_SUM };
        static constexpr const char* hardware[HARDWARE_SUM] = { "modbusTCP", "socket" };

    public:
        PartialDischarge() = delete;
        PartialDischarge(std::shared_ptr<ros::NodeHandle>& NodeHandle);
        ~PartialDischarge();

    protected:
        void init();
        void update(const ros::TimerEvent& Event);
        bool onServiceRead(whi_interfaces::WhiSrvPartialDischarge::Request& Req,
            whi_interfaces::WhiSrvPartialDischarge::Response& Res);

    protected:
        std::shared_ptr<ros::NodeHandle> node_handle_{ nullptr };
        std::unique_ptr<ros::Timer> non_realtime_loop_{ nullptr };
        ros::Duration elapsed_time_;
        std::unique_ptr<BaseSensor> sensor_{ nullptr };
        int read_length_{ 0 };
        std::unique_ptr<ros::ServiceServer> srv_read_{ nullptr };
        std::unique_ptr<ros::Publisher> pub_line_chart_2D_{ nullptr };
	};
} // namespace whi_partial_discharge_sensor
