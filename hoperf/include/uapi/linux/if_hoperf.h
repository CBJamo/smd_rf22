#ifndef _UAPI_LINUX_IF_HOPERF_H
#define _UAPI_LINUX_IF_HOPERF_H

#include <linux/types.h>

/*
 * HopeRF "Magic Constants"
 */
#define HOPERF_ALEN             1               /* Octets in one ethernet addr   */
#define HOPERF_HLEN             5               /* Total octets in header.       */
#define HOPERF_ZLEN             5               /* Min. octets in frame sans FCS */
#define HOPERF_DATA_LEN         255             /* Max. octets in payload        */
#define HOPERF_FRAME_LEN        260             /* Max. octets in frame sans FCS */
#define HOPERF_FCS_LEN          2               /* Octets in the FCS             */

/*
 * Protocol IDs
 */
#define HOPERF_P_CONTROL        0x00            /* Datalink Control packet       */
#define HOPERF_P_IP             0x01            /* Internet Protocol packet      */
#define HOPERF_P_ARP            0x02            /* Address Resolution packet     */
#define HOPERF_P_RARP           0x03            /* Reverse Addr Res packet       */
#define HOPERF_P_PAUSE          0x04            /* IEEE Pause frames. See 802.3 31B */
#define HOPERF_P_1588           0x05            /* IEEE 1588 Timesync            */
#define HOPERF_P_MACSEC         0x06            /* MACsec - See 802.1AE          */
#define HOPERF_P_WOL            0x07            /* Wake-On-Lan                   */

/*
 * HopeRF Frame Header
 */
struct hoperf_hdr
{
	unsigned char dst[1];
	unsigned char src[1];
	unsigned char id;
	unsigned char proto;
	unsigned char len;
} __attribute__((packed));


#endif /* _UAPI_LINUX_IF_HOPERF_H */
