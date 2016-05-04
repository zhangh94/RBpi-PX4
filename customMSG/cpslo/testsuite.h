/** @file
 *	@brief MAVLink comm protocol testsuite generated from cpslo.xml
 *	@see http://qgroundcontrol.org/mavlink/
 */
#ifndef CPSLO_TESTSUITE_H
#define CPSLO_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
static void mavlink_test_common(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_cpslo(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_common(system_id, component_id, last_msg);
	mavlink_test_cpslo(system_id, component_id, last_msg);
}
#endif

#include "../common/testsuite.h"


static void mavlink_test_cpslo_msg(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_cpslo_msg_t packet_in = {
		963497464,17,84,151,218,2
    };
	mavlink_cpslo_msg_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.Param4 = packet_in.Param4;
        	packet1.MissionStatus = packet_in.MissionStatus;
        	packet1.Param2 = packet_in.Param2;
        	packet1.Param3 = packet_in.Param3;
        	packet1.Param5 = packet_in.Param5;
        	packet1.mavlink_version = packet_in.mavlink_version;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_cpslo_msg_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_cpslo_msg_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_cpslo_msg_pack(system_id, component_id, &msg , packet1.MissionStatus , packet1.Param2 , packet1.Param3 , packet1.Param4 , packet1.Param5 );
	mavlink_msg_cpslo_msg_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_cpslo_msg_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.MissionStatus , packet1.Param2 , packet1.Param3 , packet1.Param4 , packet1.Param5 );
	mavlink_msg_cpslo_msg_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_cpslo_msg_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_cpslo_msg_send(MAVLINK_COMM_1 , packet1.MissionStatus , packet1.Param2 , packet1.Param3 , packet1.Param4 , packet1.Param5 );
	mavlink_msg_cpslo_msg_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_cpslo(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_cpslo_msg(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // CPSLO_TESTSUITE_H
