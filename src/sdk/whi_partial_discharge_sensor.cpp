/******************************************************************
class of partial discharge sensor

Features:
- ModbusTCP
- xxx

Written by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_partial_discharge_sensor/whi_partial_discharge_sensor.h"
#include "whi_partial_discharge_sensor/sensor_modbus.h"
#include "whi_partial_discharge_sensor/sensor_socket.h"

namespace whi_partial_discharge_sensor
{
    PartialDischarge::PartialDischarge(std::shared_ptr<ros::NodeHandle>& NodeHandle)
        : node_handle_(NodeHandle)
    {
        init();
    }

    PartialDischarge::~PartialDischarge()
    {

    }

    void PartialDischarge::init()
    {
        // params
        double frequency = 10.0;
        node_handle_->param("frequency", frequency, 10.0);
        std::string hardwareStr;
		node_handle_->param("hardware", hardwareStr, std::string(hardware[HARDWARE_MODBUS_TCP]));

        // // twist publisher
        // std::string topicTwist;
        // node_handle_->param("whi_rc_bridge/twist_topic", topicTwist, std::string("cmd_vel"));
        // pub_twist_ = std::make_unique<ros::Publisher>(
        //     node_handle_->advertise<geometry_msgs::Twist>(topicTwist, 50));

        // bridge instance
        if (hardwareStr == hardware[HARDWARE_MODBUS_TCP])
        {
            std::string addr;
            int port = 0;
            int devAddr = 0;
            node_handle_->param("modbusTCP/addr", addr, std::string());
            node_handle_->param("modbusTCP/port", port, 8000);
            node_handle_->param("modbusTCP/device_addr", devAddr, 1);
            sensor_ = std::make_unique<ModbusSensor>(addr, port, devAddr);
        }
        else if (hardwareStr == hardware[HARDWARE_SOCKET])
        {
            std::string addr;
            int port = 0;
            int devAddr = 0;
            node_handle_->param("socket/addr", addr, std::string());
            node_handle_->param("socket/port", port, 8000);
            sensor_ = std::make_unique<SocketSensor>(addr, port);
        }

        // advertise the read service
        srv_read_ = std::make_unique<ros::ServiceServer>(node_handle_->advertiseService(
            "read_pd", &PartialDischarge::onServiceRead, this));

        ros::Duration updateFreq = ros::Duration(1.0 / frequency);
		non_realtime_loop_ = std::make_unique<ros::Timer>(node_handle_->createTimer(
            updateFreq, std::bind(&PartialDischarge::update, this, std::placeholders::_1)));
    }

    void PartialDischarge::update(const ros::TimerEvent& Event)
    {
		elapsed_time_ = ros::Duration(Event.current_real - Event.last_real);
    }

    bool PartialDischarge::onServiceRead(whi_interfaces::WhiSrvRead::Request& Req,
        whi_interfaces::WhiSrvRead::Response& Res)
    {
        if (sensor_)
        {
            Res.data = sensor_->readChannel(Req.addr);

            return true;
        }
        else
        {
            return false;
        }
    }
} // namespace whi_rc_bridge
