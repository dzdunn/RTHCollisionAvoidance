#pragma once
// MESSAGE ARDUINO_DATA PACKING

#define MAVLINK_MSG_ID_ARDUINO_DATA 215

MAVPACKED(
typedef struct __mavlink_arduino_data_t {
 uint32_t distance; /*< Distance recorded by a rangefinder.*/
}) mavlink_arduino_data_t;

#define MAVLINK_MSG_ID_ARDUINO_DATA_LEN 4
#define MAVLINK_MSG_ID_ARDUINO_DATA_MIN_LEN 4
#define MAVLINK_MSG_ID_215_LEN 4
#define MAVLINK_MSG_ID_215_MIN_LEN 4

#define MAVLINK_MSG_ID_ARDUINO_DATA_CRC 111
#define MAVLINK_MSG_ID_215_CRC 111



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ARDUINO_DATA { \
    215, \
    "ARDUINO_DATA", \
    1, \
    {  { "distance", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_arduino_data_t, distance) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ARDUINO_DATA { \
    "ARDUINO_DATA", \
    1, \
    {  { "distance", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_arduino_data_t, distance) }, \
         } \
}
#endif

/**
 * @brief Pack a arduino_data message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param distance Distance recorded by a rangefinder.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_arduino_data_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ARDUINO_DATA_LEN];
    _mav_put_uint32_t(buf, 0, distance);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ARDUINO_DATA_LEN);
#else
    mavlink_arduino_data_t packet;
    packet.distance = distance;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ARDUINO_DATA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ARDUINO_DATA;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ARDUINO_DATA_MIN_LEN, MAVLINK_MSG_ID_ARDUINO_DATA_LEN, MAVLINK_MSG_ID_ARDUINO_DATA_CRC);
}

/**
 * @brief Pack a arduino_data message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param distance Distance recorded by a rangefinder.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_arduino_data_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ARDUINO_DATA_LEN];
    _mav_put_uint32_t(buf, 0, distance);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ARDUINO_DATA_LEN);
#else
    mavlink_arduino_data_t packet;
    packet.distance = distance;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ARDUINO_DATA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ARDUINO_DATA;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ARDUINO_DATA_MIN_LEN, MAVLINK_MSG_ID_ARDUINO_DATA_LEN, MAVLINK_MSG_ID_ARDUINO_DATA_CRC);
}

/**
 * @brief Encode a arduino_data struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param arduino_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_arduino_data_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_arduino_data_t* arduino_data)
{
    return mavlink_msg_arduino_data_pack(system_id, component_id, msg, arduino_data->distance);
}

/**
 * @brief Encode a arduino_data struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param arduino_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_arduino_data_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_arduino_data_t* arduino_data)
{
    return mavlink_msg_arduino_data_pack_chan(system_id, component_id, chan, msg, arduino_data->distance);
}

/**
 * @brief Send a arduino_data message
 * @param chan MAVLink channel to send the message
 *
 * @param distance Distance recorded by a rangefinder.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_arduino_data_send(mavlink_channel_t chan, uint32_t distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ARDUINO_DATA_LEN];
    _mav_put_uint32_t(buf, 0, distance);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARDUINO_DATA, buf, MAVLINK_MSG_ID_ARDUINO_DATA_MIN_LEN, MAVLINK_MSG_ID_ARDUINO_DATA_LEN, MAVLINK_MSG_ID_ARDUINO_DATA_CRC);
#else
    mavlink_arduino_data_t packet;
    packet.distance = distance;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARDUINO_DATA, (const char *)&packet, MAVLINK_MSG_ID_ARDUINO_DATA_MIN_LEN, MAVLINK_MSG_ID_ARDUINO_DATA_LEN, MAVLINK_MSG_ID_ARDUINO_DATA_CRC);
#endif
}

/**
 * @brief Send a arduino_data message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_arduino_data_send_struct(mavlink_channel_t chan, const mavlink_arduino_data_t* arduino_data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_arduino_data_send(chan, arduino_data->distance);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARDUINO_DATA, (const char *)arduino_data, MAVLINK_MSG_ID_ARDUINO_DATA_MIN_LEN, MAVLINK_MSG_ID_ARDUINO_DATA_LEN, MAVLINK_MSG_ID_ARDUINO_DATA_CRC);
#endif
}

#if MAVLINK_MSG_ID_ARDUINO_DATA_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_arduino_data_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, distance);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARDUINO_DATA, buf, MAVLINK_MSG_ID_ARDUINO_DATA_MIN_LEN, MAVLINK_MSG_ID_ARDUINO_DATA_LEN, MAVLINK_MSG_ID_ARDUINO_DATA_CRC);
#else
    mavlink_arduino_data_t *packet = (mavlink_arduino_data_t *)msgbuf;
    packet->distance = distance;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARDUINO_DATA, (const char *)packet, MAVLINK_MSG_ID_ARDUINO_DATA_MIN_LEN, MAVLINK_MSG_ID_ARDUINO_DATA_LEN, MAVLINK_MSG_ID_ARDUINO_DATA_CRC);
#endif
}
#endif

#endif

// MESSAGE ARDUINO_DATA UNPACKING


/**
 * @brief Get field distance from arduino_data message
 *
 * @return Distance recorded by a rangefinder.
 */
static inline uint32_t mavlink_msg_arduino_data_get_distance(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Decode a arduino_data message into a struct
 *
 * @param msg The message to decode
 * @param arduino_data C-struct to decode the message contents into
 */
static inline void mavlink_msg_arduino_data_decode(const mavlink_message_t* msg, mavlink_arduino_data_t* arduino_data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    arduino_data->distance = mavlink_msg_arduino_data_get_distance(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ARDUINO_DATA_LEN? msg->len : MAVLINK_MSG_ID_ARDUINO_DATA_LEN;
        memset(arduino_data, 0, MAVLINK_MSG_ID_ARDUINO_DATA_LEN);
    memcpy(arduino_data, _MAV_PAYLOAD(msg), len);
#endif
}
