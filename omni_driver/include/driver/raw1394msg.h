#pragma once


#include <libraw1394/raw1394.h>
#include <boost/shared_ptr.hpp>


/**
 * @brief Messages exchanged when calling raw1394_[read,write].
 */
class Raw1394Msg
{
public:
    const nodeaddr_t ADDR;    ///< Message address.
    const size_t     LENGTH;  ///< Message length.
    quadlet_t        data;    ///< The message data.

public:

    /**
     * @brief Constructor for Raw1394Msg.
     * @param address The channel address.
     * @param length The message length.
     * @param data The message data.
     */
    Raw1394Msg(nodeaddr_t address, size_t LENGTH, quadlet_t data);

    /**
     * @brief Constructor for Raw1394Msg.
     * @param address The channel address.
     * @param length The message length.
     */
    Raw1394Msg(nodeaddr_t address, size_t LENGTH);

    /**
     * @brief Default constructor.
     */
    Raw1394Msg();

    /**
     * @brief Returns the current data value.
     * @return The data.
     * @see getDataPtr, setData
     */
    inline quadlet_t getData() const
    {
        return data;
    }

    /**
     * @brief Sets the data to be written on the channel.
     * @param data The data.
     * @see write, getData, getDataPtr
     */
    inline void setData(quadlet_t data)
    {
        this->data = data;
    }

    /**
     * @brief Clears the data stored in this message.
     *
     * Note that this is not the same as calling setDataPtr(0). Setting the
     * pointer to zero would also attempt to access this memory position.
     */
    inline void clearData()
    {
        data = -1;
    }

    /**
     * @brief Calls libraw1394_read passing the stored address, length, and data values.
     * @param handle libraw1394 handle
     * @param node target node ide
     * @return 0 on success or -1 on failure.
     * @see libraw1394_read
     */
    int read(raw1394handle_t handle, nodeid_t node);

    /**
     * @brief Calls libraw1394_write passing the stored address, length, and data values.
     * @param handle libraw1394 handle
     * @param node target node ide
     * @return 0 on success or -1 on failure.
     * @see libraw1394_write
     */
    int write(raw1394handle_t handle, nodeid_t node);

    /**
     * @brief Same as \see{getData}
     */
    inline operator quadlet_t () const
    {
        return data;
    }

    /**
     * @brief Same as calling setData
     * @param data
     * @return
     */
    inline Raw1394Msg & operator=(quadlet_t data)
    {
        this->setData(data);
        return *this;
    }
};


typedef boost::shared_ptr<Raw1394Msg> Raw1394MsgPtr;
