#include "../include/omnifirewire.h"
#include "../include/omniethernet.h"


int main(int argc, char **argv)
{
    // Initialize ROS.
    ros::init(argc, argv, "omni");

    // Get the omni parameters.
    std::string omni_name;
    ros::param::param<std::string>("~omni_name", omni_name, "omni");
    std::string omni_type;
    ros::param::param<std::string>("~omni_type", omni_type, "firewire");
    std::string omni_serial;
    ros::param::param<std::string>("~omni_serial", omni_serial, "");
    std::string path_urdf;
    ros::param::param<std::string>("~path_urdf", path_urdf, "");
    std::string path_srdf;
    ros::param::param<std::string>("~path_srdf", path_srdf, "");

    // Declare OmniBasePtr
    OmniBasePtr omni;

    if (omni_type == "firewire")
    {
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
        omni = OmniBasePtr( new OmniFirewire(omni_serial, omni_name, path_urdf, path_srdf ) );
    }
    if (omni_type == "ethernet")
    {
        // Init the ethernet driver
        omni = OmniBasePtr( new OmniEthernet( omni_name, path_urdf, path_srdf ) );
    }

    // Connect
    omni->connect();

    // ROS Mainloop
    ros::Rate rate(50);
    while (ros::ok())
    {
        ros::spinOnce();

        omni->publishOmniState();

        // Force a reconnection if connection was lost
        if (!omni->connected())
        {
            omni->disconnect();
            omni->connect();
        }

        rate.sleep();
    }

    // Disconnect
    omni->disconnect();

    return 0;
}
