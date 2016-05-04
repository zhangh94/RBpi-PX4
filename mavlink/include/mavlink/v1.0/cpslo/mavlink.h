/** @file
 *	@brief MAVLink comm protocol built from cpslo.xml
 *	@see http://mavlink.org
 */
#ifndef MAVLINKCPSLO_H
#define MAVLINKCPSLO_H

#ifndef MAVLINK_STX
#define MAVLINK_STX 254
#endif

#ifndef MAVLINK_ENDIAN
#define MAVLINK_ENDIAN MAVLINK_LITTLE_ENDIAN
#endif

#ifndef MAVLINK_ALIGNED_FIELDS
#define MAVLINK_ALIGNED_FIELDS 1
#endif

#ifndef MAVLINK_CRC_EXTRA
#define MAVLINK_CRC_EXTRA 1
#endif

#include "version.h"
#include "cpslo.h"

#endif // MAVLINK_H
