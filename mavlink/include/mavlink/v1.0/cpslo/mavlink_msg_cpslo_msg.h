// MESSAGE CPSLO_MSG PACKING

#define MAVLINK_MSG_ID_CPSLO_MSG 150

typedef struct __mavlink_cpslo_msg_t
{
 uint32_t Param4; /*< Arbitrary field (can be redefined).*/
 uint8_t MissionStatus; /*< Status of the current search pattern mission.*/
 uint8_t Param2; /*< Field should be updated to transmit GPS cordinate if ball is found.*/
 uint8_t Param3; /*< Arbitrary field (can be redefined).*/
 uint8_t Param5; /*< Arbitrary field (can be redefined).*/
 uint8_t mavlink_version; /*< MAVLink version*/
} mavlink_cpslo_msg_t;

#define MAVLINK_MSG_ID_CPSLO_MSG_LEN 9
#define MAVLINK_MSG_ID_150_LEN 9

#define MAVLINK_MSG_ID_CPSLO_MSG_CRC 129
#define MAVLINK_MSG_ID_150_CRC 129



#define MAVLINK_MESSAGE_INFO_CPSLO_MSG { \
	"CPSLO_MSG", \
	6, \
	{  { "Param4", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_cpslo_msg_t, Param4) }, \
         { "MissionStatus", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_cpslo_msg_t, MissionStatus) }, \
         { "Param2", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_cpslo_msg_t, Param2) }, \
         { "Param3", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_cpslo_msg_t, Param3) }, \
         { "Param5", NULL, MAVLINK_TYPE_UINT8_T, 0, 7, offsetof(mavlink_cpslo_msg_t, Param5) }, \
         { "mavlink_version", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_cpslo_msg_t, mavlink_version) }, \
         } \
}


/**
 * @brief Pack a cpslo_msg message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param MissionStatus Status of the current search pattern mission.
 * @param Param2 Field should be updated to transmit GPS cordinate if ball is found.
 * @param Param3 Arbitrary field (can be redefined).
 * @param Param4 Arbitrary field (can be redefined).
 * @param Param5 Arbitrary field (can be redefined).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_cpslo_msg_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t MissionStatus, uint8_t Param2, uint8_t Param3, uint32_t Param4, uint8_t Param5)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_CPSLO_MSG_LEN];
	_mav_put_uint32_t(buf, 0, Param4);
	_mav_put_uint8_t(buf, 4, MissionStatus);
	_mav_put_uint8_t(buf, 5, Param2);
	_mav_put_uint8_t(buf, 6, Param3);
	_mav_put_uint8_t(buf, 7, Param5);
	_mav_put_uint8_t(buf, 8, 2);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CPSLO_MSG_LEN);
#else
	mavlink_cpslo_msg_t packet;
	packet.Param4 = Param4;
	packet.MissionStatus = MissionStatus;
	packet.Param2 = Param2;
	packet.Param3 = Param3;
	packet.Param5 = Param5;
	packet.mavlink_version = 2;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CPSLO_MSG_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_CPSLO_MSG;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CPSLO_MSG_LEN, MAVLINK_MSG_ID_CPSLO_MSG_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CPSLO_MSG_LEN);
#endif
}

/**
 * @brief Pack a cpslo_msg message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param MissionStatus Status of the current search pattern mission.
 * @param Param2 Field should be updated to transmit GPS cordinate if ball is found.
 * @param Param3 Arbitrary field (can be redefined).
 * @param Param4 Arbitrary field (can be redefined).
 * @param Param5 Arbitrary field (can be redefined).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_cpslo_msg_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t MissionStatus,uint8_t Param2,uint8_t Param3,uint32_t Param4,uint8_t Param5)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_CPSLO_MSG_LEN];
	_mav_put_uint32_t(buf, 0, Param4);
	_mav_put_uint8_t(buf, 4, MissionStatus);
	_mav_put_uint8_t(buf, 5, Param2);
	_mav_put_uint8_t(buf, 6, Param3);
	_mav_put_uint8_t(buf, 7, Param5);
	_mav_put_uint8_t(buf, 8, 2);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CPSLO_MSG_LEN);
#else
	mavlink_cpslo_msg_t packet;
	packet.Param4 = Param4;
	packet.MissionStatus = MissionStatus;
	packet.Param2 = Param2;
	packet.Param3 = Param3;
	packet.Param5 = Param5;
	packet.mavlink_version = 2;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CPSLO_MSG_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_CPSLO_MSG;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CPSLO_MSG_LEN, MAVLINK_MSG_ID_CPSLO_MSG_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CPSLO_MSG_LEN);
#endif
}

/**
 * @brief Encode a cpslo_msg struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param cpslo_msg C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_cpslo_msg_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_cpslo_msg_t* cpslo_msg)
{
	return mavlink_msg_cpslo_msg_pack(system_id, component_id, msg, cpslo_msg->MissionStatus, cpslo_msg->Param2, cpslo_msg->Param3, cpslo_msg->Param4, cpslo_msg->Param5);
}

/**
 * @brief Encode a cpslo_msg struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param cpslo_msg C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_cpslo_msg_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_cpslo_msg_t* cpslo_msg)
{
	return mavlink_msg_cpslo_msg_pack_chan(system_id, component_id, chan, msg, cpslo_msg->MissionStatus, cpslo_msg->Param2, cpslo_msg->Param3, cpslo_msg->Param4, cpslo_msg->Param5);
}

/**
 * @brief Send a cpslo_msg message
 * @param chan MAVLink channel to send the message
 *
 * @param MissionStatus Status of the current search pattern mission.
 * @param Param2 Field should be updated to transmit GPS cordinate if ball is found.
 * @param Param3 Arbitrary field (can be redefined).
 * @param Param4 Arbitrary field (can be redefined).
 * @param Param5 Arbitrary field (can be redefined).
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_cpslo_msg_send(mavlink_channel_t chan, uint8_t MissionStatus, uint8_t Param2, uint8_t Param3, uint32_t Param4, uint8_t Param5)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_CPSLO_MSG_LEN];
	_mav_put_uint32_t(buf, 0, Param4);
	_mav_put_uint8_t(buf, 4, MissionStatus);
	_mav_put_uint8_t(buf, 5, Param2);
	_mav_put_uint8_t(buf, 6, Param3);
	_mav_put_uint8_t(buf, 7, Param5);
	_mav_put_uint8_t(buf, 8, 2);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CPSLO_MSG, buf, MAVLINK_MSG_ID_CPSLO_MSG_LEN, MAVLINK_MSG_ID_CPSLO_MSG_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CPSLO_MSG, buf, MAVLINK_MSG_ID_CPSLO_MSG_LEN);
#endif
#else
	mavlink_cpslo_msg_t packet;
	packet.Param4 = Param4;
	packet.MissionStatus = MissionStatus;
	packet.Param2 = Param2;
	packet.Param3 = Param3;
	packet.Param5 = Param5;
	packet.mavlink_version = 2;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CPSLO_MSG, (const char *)&packet, MAVLINK_MSG_ID_CPSLO_MSG_LEN, MAVLINK_MSG_ID_CPSLO_MSG_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CPSLO_MSG, (const char *)&packet, MAVLINK_MSG_ID_CPSLO_MSG_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_CPSLO_MSG_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_cpslo_msg_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t MissionStatus, uint8_t Param2, uint8_t Param3, uint32_t Param4, uint8_t Param5)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint32_t(buf, 0, Param4);
	_mav_put_uint8_t(buf, 4, MissionStatus);
	_mav_put_uint8_t(buf, 5, Param2);
	_mav_put_uint8_t(buf, 6, Param3);
	_mav_put_uint8_t(buf, 7, Param5);
	_mav_put_uint8_t(buf, 8, 2);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CPSLO_MSG, buf, MAVLINK_MSG_ID_CPSLO_MSG_LEN, MAVLINK_MSG_ID_CPSLO_MSG_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CPSLO_MSG, buf, MAVLINK_MSG_ID_CPSLO_MSG_LEN);
#endif
#else
	mavlink_cpslo_msg_t *packet = (mavlink_cpslo_msg_t *)msgbuf;
	packet->Param4 = Param4;
	packet->MissionStatus = MissionStatus;
	packet->Param2 = Param2;
	packet->Param3 = Param3;
	packet->Param5 = Param5;
	packet->mavlink_version = 2;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CPSLO_MSG, (const char *)packet, MAVLINK_MSG_ID_CPSLO_MSG_LEN, MAVLINK_MSG_ID_CPSLO_MSG_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CPSLO_MSG, (const char *)packet, MAVLINK_MSG_ID_CPSLO_MSG_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE CPSLO_MSG UNPACKING


/**
 * @brief Get field MissionStatus from cpslo_msg message
 *
 * @return Status of the current search pattern mission.
 */
static inline uint8_t mavlink_msg_cpslo_msg_get_MissionStatus(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field Param2 from cpslo_msg message
 *
 * @return Field should be updated to transmit GPS cordinate if ball is found.
 */
static inline uint8_t mavlink_msg_cpslo_msg_get_Param2(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field Param3 from cpslo_msg message
 *
 * @return Arbitrary field (can be redefined).
 */
static inline uint8_t mavlink_msg_cpslo_msg_get_Param3(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Get field Param4 from cpslo_msg message
 *
 * @return Arbitrary field (can be redefined).
 */
static inline uint32_t mavlink_msg_cpslo_msg_get_Param4(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field Param5 from cpslo_msg message
 *
 * @return Arbitrary field (can be redefined).
 */
static inline uint8_t mavlink_msg_cpslo_msg_get_Param5(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  7);
}

/**
 * @brief Get field mavlink_version from cpslo_msg message
 *
 * @return MAVLink version
 */
static inline uint8_t mavlink_msg_cpslo_msg_get_mavlink_version(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Decode a cpslo_msg message into a struct
 *
 * @param msg The message to decode
 * @param cpslo_msg C-struct to decode the message contents into
 */
static inline void mavlink_msg_cpslo_msg_decode(const mavlink_message_t* msg, mavlink_cpslo_msg_t* cpslo_msg)
{
#if MAVLINK_NEED_BYTE_SWAP
	cpslo_msg->Param4 = mavlink_msg_cpslo_msg_get_Param4(msg);
	cpslo_msg->MissionStatus = mavlink_msg_cpslo_msg_get_MissionStatus(msg);
	cpslo_msg->Param2 = mavlink_msg_cpslo_msg_get_Param2(msg);
	cpslo_msg->Param3 = mavlink_msg_cpslo_msg_get_Param3(msg);
	cpslo_msg->Param5 = mavlink_msg_cpslo_msg_get_Param5(msg);
	cpslo_msg->mavlink_version = mavlink_msg_cpslo_msg_get_mavlink_version(msg);
#else
	memcpy(cpslo_msg, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_CPSLO_MSG_LEN);
#endif
}
