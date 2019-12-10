#include "../include/raw1394msg.h"

// Included to use the function ntohl
#include <arpa/inet.h>


Raw1394Msg::Raw1394Msg(nodeaddr_t address, size_t length, quadlet_t data)
    : ADDR(address), LENGTH(length), data(data)
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
    int status = raw1394_read(handle, node, ADDR, LENGTH, &data);

    // Converts the byte order from the network channel to the one
    // used in the host machine
    data = ntohl(data);

    return status;
}


int Raw1394Msg::write(raw1394handle_t handle, nodeid_t node)
{
    return raw1394_write(handle, node, ADDR, LENGTH, &data);
}
