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
    static float convertIeee754ToDecimal(uint32_t Raw)
    {
        float sign = Raw >> 31 == 0 ? 1.0 : -1.0;
        uint32_t mantissa = (Raw & 0x7FFFFF) | 0x800000;
        int32_t exp = ((Raw >> 23) & 0xFF) - 127 - 23;
        return sign * mantissa * pow(2.0, exp);
    }

    PartialDischarge::PartialDischarge(std::shared_ptr<ros::NodeHandle>& NodeHandle)
        : node_handle_(NodeHandle)
    {
        init();
    }

    PartialDischarge::~PartialDischarge()
    {
        if (sensor_)
        {
            sensor_->close();
        }
    }

    void PartialDischarge::init()
    {
        // params
        double frequency = 10.0;
        node_handle_->param("frequency", frequency, 10.0);
        node_handle_->param("data_length", read_length_, 0);
        std::string hardwareStr;
		node_handle_->param("hardware", hardwareStr, std::string(hardware[HARDWARE_MODBUS_TCP]));

        // instance
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

    bool PartialDischarge::onServiceRead(whi_interfaces::WhiSrvReadDischarge::Request& Req,
        whi_interfaces::WhiSrvReadDischarge::Response& Res)
    {
        if (sensor_)
        {
            auto data = sensor_->readChannel(read_length_, Req.addr);
            if (!data.empty())
            {
#ifdef DEBUG
                std::cout << "returned data:" << std::endl;
                for (const auto& it : data)
                {
                    std::cout << std::hex << int(it) << ",";
                }
                std::cout << std::dec << " with size " << sizeof(data) << std::endl;
#endif

                std::vector<std::array<float, 7>> translated;
                for (int i = 3; i < read_length_; i = i + 28)
                {
                    std::array<float, 7> channelTranslated;
                    for (int j = 0; j < channelTranslated.size(); ++j)
                    {
                        channelTranslated[j] = convertIeee754ToDecimal(uint32_t(
                            data[i + j * 4] << 24 | data[i + j * 4 + 1] << 16 | data[i + j * 4 + 2] << 8 | data[i + j * 4 + 3]));
                    }
                    translated.push_back(channelTranslated);
                }

                for (int i = 0; i < translated.size(); ++i)
                {
                    whi_interfaces::WhiPartialDischarge channel;
                    channel.channel = uint8_t(i);
                    channel.peak = translated[i][0];
                    channel.average = translated[i][1];
                    channel.noise = translated[i][2];
                    channel.phase = translated[i][3];
                    channel.count = int(translated[i][4]);
                    channel.cycle_count = int(translated[i][5]);
                    channel.state = uint8_t(translated[i][6]);       

                    Res.data.push_back(channel);
                }
            
                return true;
            }
            else
            {
                return false;
            }
        }
        else
        {
            return false;
        }
    }
} // namespace whi_rc_bridge
