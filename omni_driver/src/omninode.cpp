#include "../include/omnibase.h"
#include "../include/omnifirewire.h"


int main(int argc, char **argv)
{
    // Initialize ROS.
    ros::init(argc, argv, "omni");

    // Get the omni parameters.
    std::string omni_name;
    ros::param::param<std::string>("~omni_name", omni_name, "");
    std::string omni_serial;
    ros::param::param<std::string>("~omni_serial", omni_serial, "");

    // List the serial number of each Phantom Omni connected. This is only
    // used to acquire the serial number if none was passed to the program
    const std::vector<OmniFirewire::OmniInfo>& available_devices_list = OmniFirewire::enumerate_omnis();
    std::ostringstream serial_numbers;
    for (size_t i = 0; i < available_devices_list.size(); i++) {
        serial_numbers << available_devices_list[i].serial << " ";
    }
    if (available_devices_list.size() > 0)
    {
        ROS_INFO("Phantom Omni Serial Numbers: %s", serial_numbers.str().c_str());
        if (omni_serial.empty())
        {
            ROS_INFO("Will connect to the first device since no serial number was passed.");
            omni_serial = available_devices_list[0].serial;
        }
    }
    else
    {
        ROS_ERROR("No firewire devices found.");
        return -1;
    }

    // Init the firewire driver
    OmniFirewirePtr omni( new OmniFirewire(omni_serial, omni_name) );

    // Connect
    omni->connect();

    // ROS Mainloop
    ros::Rate rate(100);
    while (ros::ok())
    {
        ros::spinOnce();

        omni->publishOmniState();

        rate.sleep();
    }

    // Disconnect
    omni->disconnect();

    return 0;
}
