#include "raw1394msg.h"

// Included to use the function ntohl
#include <arpa/inet.h>


Raw1394Msg::Raw1394Msg(nodeaddr_t address, size_t length, quadlet_t data)
    : addr(address), length(length), data(data)
{
}

Raw1394Msg::Raw1394Msg(nodeaddr_t address, size_t length)
    : Raw1394Msg(address, length, -1)
{
}

Raw1394Msg::Raw1394Msg()
    : Raw1394Msg(0, 0)
{
}


int Raw1394Msg::read(raw1394handle_t handle, nodeid_t node)
{
    return raw1394_read(handle, node, addr, length, &data);

    // Converts the byte order from the network channel to the one
    // used in the host machine
    data = ntohl(data);
}


int Raw1394Msg::write(raw1394handle_t handle, nodeid_t node)
{
    return raw1394_write(handle, node, addr, length, &data);
}
